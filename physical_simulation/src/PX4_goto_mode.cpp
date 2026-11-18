#include "physical_simulation/PX4_goto_mode.hpp"
#include <algorithm>
#include <cmath>
#include <optional>

static const std::string kModeName = "Trajectory Mode";

MyGotoMode::MyGotoMode(rclcpp::Node &node)
    : ModeBase(node, kModeName)
{
    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
    _vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);
}

void MyGotoMode::setWaypoints(const std::vector<WaypointTask> &waypoints)
{
    _waypoints = waypoints;

    // 第 0 點時間戳防呆
    if (!_waypoints.empty())
    {
        const float wp0_dist_m = _waypoints[0].target_offset_ned_m.norm();
        if (_waypoints[0].desired_duration_s > 15.0f && wp0_dist_m < 100.0f)
        {
            _waypoints[0].desired_duration_s = std::clamp(wp0_dist_m / 3.0f, 1.5f, 10.0f);
        }
    }
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

void MyGotoMode::updateSetpoint(float dt_s)
{
    (void)dt_s;

    if (!_start_position_set)
    {
        _start_position_m = _vehicle_local_position->positionNed();
        _start_position_set = true;
    }

    switch (_state)
    {
    case State::SettlingAtStart:
    {
        _trajectory_setpoint->updatePosition(_start_position_m);
        if (positionReached(_start_position_m))
        {
            _state = State::ExecutingWaypoints;
            _segment_initialized = false;
        }
    }
    break;

    case State::ExecutingWaypoints:
    {
        if (_current_wp_index >= _waypoints.size())
        {
            _trajectory_setpoint->update(Eigen::Vector3f::Zero());
            completed(px4_ros2::Result::Success);
            return;
        }

        const WaypointTask &task = _waypoints[_current_wp_index];
        const Eigen::Vector3f target_position_m = _start_position_m + task.target_offset_ned_m;

        if (!_segment_initialized)
        {
            initCurrentSegment(target_position_m);
            _segment_initialized = true;
        }

        const Eigen::Vector3f current_pos_m = _vehicle_local_position->positionNed();
        const Eigen::Vector2f horiz_vec = (target_position_m - current_pos_m).head<2>();
        const float horiz_dist_m = horiz_vec.norm();

        // 1. 中間航點直接以該點期望速度指向目標，加減速與轉彎弧線全交由 PX4 飛控計算；
        //    僅在最後一個終點做平滑減速停靠。
        const bool is_final_waypoint = (_current_wp_index == _waypoints.size() - 1);
        const float target_horiz_speed = is_final_waypoint
                                             ? std::min(_max_horizontal_velocity_m_s, 1.5f * horiz_dist_m)
                                             : _max_horizontal_velocity_m_s;

        Eigen::Vector2f vel_xy_ned = Eigen::Vector2f::Zero();
        if (horiz_dist_m > 0.05f)
        {
            vel_xy_ned = (horiz_vec / horiz_dist_m) * target_horiz_speed;
        }

        // 2. 垂直高度比例控制
        const float z_err_m = target_position_m.z() - current_pos_m.z();
        const float vel_z_ned = std::clamp(
            1.5f * z_err_m, -_max_vertical_velocity_m_s, _max_vertical_velocity_m_s);

        const Eigen::Vector3f vel_cmd_ned(vel_xy_ned.x(), vel_xy_ned.y(), vel_z_ned);

        if (!task.face_direction_of_travel || horiz_dist_m < 0.1f)
        {
            _trajectory_setpoint->update(vel_cmd_ned, std::nullopt, std::nullopt);
        }
        else
        {
            const float heading_target_rad = std::atan2(horiz_vec.y(), horiz_vec.x());
            _trajectory_setpoint->update(vel_cmd_ned, std::nullopt, heading_target_rad);
        }

        // 3. 抵達判定
        if (positionReached(target_position_m))
        {
            _reports.push_back(recordAndPrintReport(_current_wp_index, target_position_m));
            _current_wp_index++;
            _segment_initialized = false;
        }
    }
    break;
    }
}

void MyGotoMode::initCurrentSegment(const Eigen::Vector3f &target_position_m)
{
    _segment_start_pos_m = _vehicle_local_position->positionNed();
    _segment_start_time = node().get_clock()->now();

    const WaypointTask &task = _waypoints[_current_wp_index];
    const Eigen::Vector3f delta = target_position_m - _segment_start_pos_m;
    const float horiz_distance_m = delta.head<2>().norm();
    const float vert_distance_m = std::fabs(delta.z());
    const float duration_s = std::max(task.desired_duration_s, 0.5f);

    _max_horizontal_velocity_m_s = std::clamp(
        (horiz_distance_m / duration_s) * kRampCompensation, 0.5f, 17.0f);

    _max_vertical_velocity_m_s = std::clamp(
        (vert_distance_m / duration_s) * kRampCompensation, 0.3f, 5.0f);
}

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

WaypointTelemetryReport MyGotoMode::recordAndPrintReport(
    size_t wp_index, const Eigen::Vector3f &target_position_m)
{
    const rclcpp::Time now = node().get_clock()->now();
    WaypointTelemetryReport report;

    report.waypoint_index = wp_index;
    report.target_position_ned_m = target_position_m;
    report.actual_position_ned_m = _vehicle_local_position->positionNed();
    report.position_error_ned_m = report.actual_position_ned_m - target_position_m;
    report.position_error_norm_m = report.position_error_ned_m.norm();
    report.actual_velocity_ned_m_s = _vehicle_local_position->velocityNed();
    report.actual_speed_norm_m_s = report.actual_velocity_ned_m_s.norm();
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
                "  期望抵達時間       : %.2f s (目標速度: %.2f m/s)\n"
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