#include "physical_simulation/PX4_goto_mode.hpp"
#include <algorithm>
#include <cmath>

static const std::string kModeName = "Autonomous Goto Mode";

MyGotoMode::MyGotoMode(rclcpp::Node &node)
    : ModeBase(node, kModeName) // 不使用 preventArming，因為 Executor 可能需要自動起飛
{
    // 實例化介面並綁定到當前 Mode (*this)
    // 底層會使用外部傳入的 rclcpp::Node 去建立必要的 Pub/Sub，支援 Intra-process Zero-copy
    _goto_setpoint = std::make_shared<px4_ros2::MulticopterGotoSetpointType>(*this);
    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
    _vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);

    // 預設一組測試航點 (若 Action Server 未傳入時備用)
    _waypoints = {
        {Eigen::Vector3f{15.f, 0.f, 0.f}, 5.0f, true},
        {Eigen::Vector3f{15.f, 20.f, -2.f}, 6.0f, true},
        {Eigen::Vector3f{0.f, 0.f, 0.f}, 7.0f, true}};
}

// 供 Executor 轉發的 Getter/Setter
void MyGotoMode::setWaypoints(const std::vector<WaypointTask> &waypoints)
{
    _waypoints = waypoints;
}
const std::vector<WaypointTelemetryReport> &MyGotoMode::getTelemetryReports() const
{
    return _reports;
}
Eigen::Vector3f MyGotoMode::getCurrentPositionNed() const
{
    return _vehicle_local_position->positionNed();
}
Eigen::Vector3f MyGotoMode::getCurrentVelocityNed() const
{
    return _vehicle_local_position->velocityNed();
}

void MyGotoMode::onActivate()
{
    _state = State::SettlingAtStart;
    _start_position_set = false;
    _current_wp_index = 0;
    _segment_initialized = false;
    _reports.clear();
    _reports.reserve(_waypoints.size());
}

void MyGotoMode::onDeactivate() {}

// 此迴圈會在模式啟動後由 PX4 介面庫依賴 rclcpp Timer 高頻觸發
void MyGotoMode::updateSetpoint(float dt_s)
{
    // 延遲至第一個控制週期才鎖定基準位置，確保取得最新鮮的 Odometry 數據
    if (!_start_position_set)
    {
        _start_position_m = _vehicle_local_position->positionNed();
        _start_position_set = true;
    }

    switch (_state)
    {
    case State::SettlingAtStart:
    {
        // 階段 1：將目標鎖死在當前位置，等待無人機穩定 (避免切換 Mode 瞬間發生姿態震盪)
        _goto_setpoint->update(_start_position_m);
        if (positionReached(_start_position_m))
        {
            _state = State::ExecutingWaypoints;
            _segment_initialized = false;
        }
    }
    break;

    case State::ExecutingWaypoints:
    {
        // 檢查是否所有路徑點皆已完成
        if (_current_wp_index >= _waypoints.size())
        {
            completed(px4_ros2::Result::Success); // 通知 Executor 此 Mode 任務結束
            return;
        }

        // 提取當前任務與絕對目標座標 (基準點 + 偏移量)
        const WaypointTask &task = _waypoints[_current_wp_index];
        const Eigen::Vector3f target_position_m = _start_position_m + task.target_offset_ned_m;

        // 若為新航段，執行初始化計算 (時間轉速度)
        if (!_segment_initialized)
        {
            initCurrentSegment(target_position_m);
            _segment_initialized = true;
        }

        // 讀取當前即時位置並計算水平向量與距離
        const Eigen::Vector3f current_pos_m = _vehicle_local_position->positionNed();
        const Eigen::Vector2f horiz_vec = (target_position_m - current_pos_m).head<2>();
        const float horiz_dist_m = horiz_vec.norm();

        // [官方防護機制] 當水平距離極短 (<0.1m) 時，反正切函數易生奇異點，
        // 此時送出 std::nullopt (不指定 heading) 以防抵達瞬間機頭劇烈抖動。
        if (!task.face_direction_of_travel || horiz_dist_m < 0.1f)
        {
            _goto_setpoint->update(
                target_position_m,
                std::nullopt,
                _max_horizontal_velocity_m_s,
                _max_vertical_velocity_m_s);
        }
        else
        {
            const float heading_target_rad = std::atan2(horiz_vec.y(), horiz_vec.x());
            _goto_setpoint->update(
                target_position_m,
                heading_target_rad,
                _max_horizontal_velocity_m_s,
                _max_vertical_velocity_m_s);
        }

        // 檢查當前距離誤差是否小於閾值 (抵達判定)
        if (positionReached(target_position_m))
        {
            _reports.push_back(recordAndPrintReport(_current_wp_index, target_position_m));
            _current_wp_index++;          // 切換至下一個點
            _segment_initialized = false; // 觸發下一個點的速度重算
        }
    }
    break;
    }
}

// 核心邏輯：將期望時間轉換為速度約束 (軟性時間同步)
void MyGotoMode::initCurrentSegment(const Eigen::Vector3f &target_position_m)
{
    _segment_start_pos_m = _vehicle_local_position->positionNed();
    _segment_start_time = node().get_clock()->now();

    const WaypointTask &task = _waypoints[_current_wp_index];
    const Eigen::Vector3f delta = target_position_m - _segment_start_pos_m;
    const float horiz_distance_m = delta.head<2>().norm();
    const float vert_distance_m = std::fabs(delta.z());

    // 避免除以 0，設定 0.5s 保護下限
    const float duration_s = std::max(task.desired_duration_s, 0.5f);

    // 速度 = (距離 / 期望時間) * 補償係數
    // 使用 std::clamp 設定合理物理極限，避免算出過低(蠕動)或過高(失控)的限速
    _max_horizontal_velocity_m_s = std::clamp(
        (horiz_distance_m / duration_s) * kRampCompensation, 0.5f, 12.0f);
    _max_vertical_velocity_m_s = std::clamp(
        (vert_distance_m / duration_s) * kRampCompensation, 0.3f, 4.0f);

    // RCLCPP_INFO(node().get_logger(),
    //             "[WP #%zu 啟動] 目標 NED: [%.2f, %.2f, %.2f] m | 期望時間: %.2f s | 換算限速: 水平 %.2f m/s, 垂直 %.2f m/s",
    //             _current_wp_index,
    //             target_position_m.x(), target_position_m.y(), target_position_m.z(),
    //             duration_s,
    //             _max_horizontal_velocity_m_s, _max_vertical_velocity_m_s);
}

// 抵達判定：計算當前位置與目標位置的 3D 歐幾里得距離
bool MyGotoMode::positionReached(const Eigen::Vector3f &target_position_m) const
{
    const float position_error =
        (target_position_m - _vehicle_local_position->positionNed()).norm();
    return position_error < kPositionErrorThreshold;
}

bool MyGotoMode::headingReached(float target_heading_rad) const
{
    const float heading_error_wrapped =
        px4_ros2::wrapPi(target_heading_rad - _vehicle_attitude->yaw());
    return std::fabs(heading_error_wrapped) < kHeadingErrorThreshold;
}

// 抵達結算：記錄並印出數據供實驗 Profiling 分析
WaypointTelemetryReport MyGotoMode::recordAndPrintReport(
    size_t wp_index, const Eigen::Vector3f &target_position_m)
{
    const rclcpp::Time now = node().get_clock()->now();
    WaypointTelemetryReport report;

    // 填寫報告資料
    report.waypoint_index = wp_index;
    report.target_position_ned_m = target_position_m;
    report.actual_position_ned_m = _vehicle_local_position->positionNed();
    report.position_error_ned_m = report.actual_position_ned_m - target_position_m;
    report.position_error_norm_m = report.position_error_ned_m.norm();
    report.actual_velocity_ned_m_s = _vehicle_local_position->velocityNed();
    report.actual_speed_norm_m_s = report.actual_velocity_ned_m_s.norm();
    report.actual_yaw_deg = px4_ros2::radToDeg(_vehicle_attitude->yaw());
    report.configured_max_horiz_speed_m_s = _max_horizontal_velocity_m_s;
    report.desired_duration_s = _waypoints[wp_index].desired_duration_s;
    report.actual_duration_s = (now - _segment_start_time).seconds();
    report.time_error_s = report.actual_duration_s - report.desired_duration_s;

    RCLCPP_INFO(node().get_logger(),
                "\n================ [路徑點 #%zu 抵達遙測結算報告] ================\n"
                "  指定目標位置 (NED) : [%.3f, %.3f, %.3f] m\n"
                "  實際抵達位置 (NED) : [%.3f, %.3f, %.3f] m\n"
                "  各軸位置誤差 (XYZ) : [%.3f, %.3f, %.3f] m (總歐氏誤差: %.3f m)\n"
                "  抵達瞬間速度 (NED) : [%.3f, %.3f, %.3f] m/s (合速度: %.3f m/s)\n"
                "  期望抵達時間       : %.2f s (換算水平限速: %.2f m/s)\n"
                "  實際飛行時間       : %.2f s (時間誤差: %+.2f s)\n"
                "==============================================================",
                report.waypoint_index,
                report.target_position_ned_m.x(), report.target_position_ned_m.y(), report.target_position_ned_m.z(),
                report.actual_position_ned_m.x(), report.actual_position_ned_m.y(), report.actual_position_ned_m.z(),
                report.position_error_ned_m.x(), report.position_error_ned_m.y(), report.position_error_ned_m.z(),
                report.position_error_norm_m,
                report.actual_velocity_ned_m_s.x(), report.actual_velocity_ned_m_s.y(), report.actual_velocity_ned_m_s.z(),
                report.actual_speed_norm_m_s,
                report.desired_duration_s, report.configured_max_horiz_speed_m_s,
                report.actual_duration_s, report.time_error_s);

    return report;
}