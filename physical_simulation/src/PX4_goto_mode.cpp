#include "physical_simulation/PX4_goto_mode.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

static const std::string kModeName = "Trajectory Mode";

MyGotoMode::MyGotoMode(rclcpp::Node &node) : ModeBase(node, kModeName) {
  _trajectory_setpoint =
      std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
  _vehicle_local_position =
      std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
  _vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);

  UAV_ID = static_cast<uint16_t>(node.get_parameter("UAV_id").as_int());
  file_open();
}

void MyGotoMode::setWaypoints(const std::vector<WaypointTask> &waypoints) {
  _waypoints = waypoints;

  // 第 0 點時間戳防呆
  if (!_waypoints.empty()) {
    const float wp0_dist_m = _waypoints[0].target_offset_ned_m.norm();
    if (_waypoints[0].desired_duration_s > 15.0f && wp0_dist_m < 100.0f) {
      _waypoints[0].desired_duration_s =
          std::clamp(wp0_dist_m / 3.0f, 1.5f, 10.0f);
    }
  }
}

const std::vector<WaypointTelemetryReport> &MyGotoMode::getTelemetryReports()
    const {
  return _reports;
}
Eigen::Vector3f MyGotoMode::getCurrentPositionNed() const {
  return _vehicle_local_position->positionNed();
}
Eigen::Vector3f MyGotoMode::getCurrentVelocityNed() const {
  return _vehicle_local_position->velocityNed();
}

void MyGotoMode::onActivate() {
  _state = State::SettlingAtStart;
  _start_position_set = false;
  _current_wp_index = 0;
  _segment_initialized = false;
  _reports.clear();
  _reports.reserve(_waypoints.size());
}

void MyGotoMode::onDeactivate() {}

void MyGotoMode::updateSetpoint(float dt_s) {
  (void)dt_s;

  if (!_start_position_set) {
    _start_position_m = _vehicle_local_position->positionNed();
    _start_position_set = true;
  }

  switch (_state) {
    case State::SettlingAtStart: {
      _trajectory_setpoint->updatePosition(_start_position_m);
      if (positionReached(_start_position_m)) {
        _state = State::ExecutingWaypoints;
        _segment_initialized = false;
      }
    } break;

    case State::ExecutingWaypoints: {
      if (_current_wp_index >= _waypoints.size()) {
        _trajectory_setpoint->update(Eigen::Vector3f::Zero());
        completed(px4_ros2::Result::Success);
        return;
      }

      const WaypointTask &task = _waypoints[_current_wp_index];
      const Eigen::Vector3f target_position_m =
          _start_position_m + task.target_offset_ned_m;

      if (!_segment_initialized) {
        initCurrentSegment(target_position_m);
        _segment_initialized = true;
      }

      // 物理速度上限（請對齊 PX4 參數 MPC_XY_VEL_MAX / MPC_Z_VEL_MAX_UP, DN）
      // 必須高於軌跡標稱速度，無人機落後時才有追趕餘裕
      constexpr float kPhysicalMaxHorizSpeed = 17.0f;
      constexpr float kPhysicalMaxVertSpeed = 5.0f;

      // 剩餘時間下限保護：避免超時或接近 0 秒時除以零暴衝或速度反向
      constexpr float kMinHorizTimeRemS = 0.25f;
      constexpr float kMinVertTimeRemS = 0.40f;

      const rclcpp::Time now = node().get_clock()->now();
      const float desired_duration_s = std::max(task.desired_duration_s, 0.2f);
      const float elapsed_s =
          static_cast<float>((now - _segment_start_time).seconds());
      const float remaining_time_s = desired_duration_s - elapsed_s;

      const Eigen::Vector3f current_pos_m =
          _vehicle_local_position->positionNed();
      const Eigen::Vector3f pos_err_ned = target_position_m - current_pos_m;
      const Eigen::Vector2f horiz_vec = pos_err_ned.head<2>();
      const float horiz_dist_m = horiz_vec.norm();
      const float z_err_m = pos_err_ned.z();

      const bool is_final_waypoint =
          (_current_wp_index == _waypoints.size() - 1);

      // 1. 水平速度：剩餘水平距離 / 剩餘時間
      //    落後自動加速追趕、超前自動放慢；最後一點額外套用減速包絡線避免過衝
      const float eff_horiz_rem_s =
          std::max(remaining_time_s, kMinHorizTimeRemS);
      float target_horiz_speed = horiz_dist_m / eff_horiz_rem_s;
      if (is_final_waypoint) {
        target_horiz_speed = std::min(target_horiz_speed, 1.5f * horiz_dist_m);
      }
      target_horiz_speed = std::min(target_horiz_speed, kPhysicalMaxHorizSpeed);

      Eigen::Vector2f vel_xy_ned = Eigen::Vector2f::Zero();
      if (horiz_dist_m > 0.05f) {
        vel_xy_ned = (horiz_vec / horiz_dist_m) * target_horiz_speed;
      }

      // 2. 垂直速度：剩餘高度差 / 剩餘時間，確保三軸同步抵達
      const float eff_vert_rem_s = std::max(remaining_time_s, kMinVertTimeRemS);
      float vel_z_ned = z_err_m / eff_vert_rem_s;
      if (is_final_waypoint) {
        const float final_z_limit = 1.5f * std::fabs(z_err_m);
        vel_z_ned = std::clamp(vel_z_ned, -final_z_limit, final_z_limit);
      }
      vel_z_ned =
          std::clamp(vel_z_ned, -kPhysicalMaxVertSpeed, kPhysicalMaxVertSpeed);

      const Eigen::Vector3f vel_cmd_ned(vel_xy_ned.x(), vel_xy_ned.y(),
                                        vel_z_ned);

      if (!task.face_direction_of_travel || horiz_dist_m < 0.1f) {
        _trajectory_setpoint->update(vel_cmd_ned, std::nullopt, std::nullopt);
      } else {
        const float heading_target_rad =
            std::atan2(horiz_vec.y(), horiz_vec.x());
        _trajectory_setpoint->update(vel_cmd_ned, std::nullopt,
                                     heading_target_rad);
      }

      // 3. 抵達判定
      if (positionReached(target_position_m)) {
        _reports.push_back(
            recordAndPrintReport(_current_wp_index, target_position_m));
        _current_wp_index++;
        _segment_initialized = false;
      }
    } break;
  }
}

void MyGotoMode::initCurrentSegment(const Eigen::Vector3f &target_position_m) {
  _segment_start_pos_m = _vehicle_local_position->positionNed();
  _segment_start_time = node().get_clock()->now();

  const WaypointTask &task = _waypoints[_current_wp_index];
  const Eigen::Vector3f delta = target_position_m - _segment_start_pos_m;
  const float horiz_distance_m = delta.head<2>().norm();
  const float vert_distance_m = std::fabs(delta.z());
  const float duration_s = std::max(task.desired_duration_s, 0.2f);

  // 改為記錄該航段的理論標稱平均速度（供遙測報告與 CSV 記錄用）
  // 閉迴路時間控制會自動補償加減速落後
  _max_horizontal_velocity_m_s =
      std::clamp(horiz_distance_m / duration_s, 0.0f, 17.0f);
  _max_vertical_velocity_m_s =
      std::clamp(vert_distance_m / duration_s, 0.0f, 5.0f);
}

bool MyGotoMode::positionReached(
    const Eigen::Vector3f &target_position_m) const {
  const float position_error =
      (target_position_m - _vehicle_local_position->positionNed()).norm();
  return position_error < kPositionErrorThreshold;
}

bool MyGotoMode::headingReached(float target_heading_rad) const {
  const float heading_error_wrapped =
      px4_ros2::wrapPi(target_heading_rad - _vehicle_attitude->yaw());
  return std::fabs(heading_error_wrapped) < kHeadingErrorThreshold;
}

WaypointTelemetryReport MyGotoMode::recordAndPrintReport(
    size_t wp_index, const Eigen::Vector3f &target_position_m) {
  const rclcpp::Time now = node().get_clock()->now();
  WaypointTelemetryReport report;

  report.waypoint_index = wp_index;
  report.target_position_ned_m = target_position_m;
  report.actual_position_ned_m = _vehicle_local_position->positionNed();
  report.position_error_ned_m =
      report.actual_position_ned_m - target_position_m;
  report.position_error_norm_m = report.position_error_ned_m.norm();
  report.actual_velocity_ned_m_s = _vehicle_local_position->velocityNed();
  report.actual_speed_norm_m_s = report.actual_velocity_ned_m_s.norm();
  report.configured_max_horiz_speed_m_s = _max_horizontal_velocity_m_s;
  report.desired_duration_s = _waypoints[wp_index].desired_duration_s;
  report.actual_duration_s = (now - _segment_start_time).seconds();
  report.time_error_s = report.actual_duration_s - report.desired_duration_s;

  // RCLCPP_INFO(
  //     node().get_logger(),
  //     "\n================ [路徑點 #%zu 抵達遙測結算報告] ================\n"
  //     "  指定目標位置 (NED) : [%.3f, %.3f, %.3f] m\n"
  //     "  實際抵達位置 (NED) : [%.3f, %.3f, %.3f] m\n"
  //     "  各軸位置誤差 (XYZ) : [%.3f, %.3f, %.3f] m (總歐氏誤差: %.3f m)\n"
  //     "  抵達瞬間速度 (NED) : [%.3f, %.3f, %.3f] m/s (合速度: %.3f m/s)\n"
  //     "  期望抵達時間       : %.2f s (目標速度: %.2f m/s)\n"
  //     "  實際飛行時間       : %.2f s (時間誤差: %+.2f s)\n"
  //     "==============================================================",
  //     report.waypoint_index, report.target_position_ned_m.x(),
  //     report.target_position_ned_m.y(), report.target_position_ned_m.z(),
  //     report.actual_position_ned_m.x(), report.actual_position_ned_m.y(),
  //     report.actual_position_ned_m.z(), report.position_error_ned_m.x(),
  //     report.position_error_ned_m.y(), report.position_error_ned_m.z(),
  //     report.position_error_norm_m, report.actual_velocity_ned_m_s.x(),
  //     report.actual_velocity_ned_m_s.y(), report.actual_velocity_ned_m_s.z(),
  //     report.actual_speed_norm_m_s, report.desired_duration_s,
  //     report.configured_max_horiz_speed_m_s, report.actual_duration_s,
  //     report.time_error_s);

  if (telemetry_csv_.is_open()) {
    telemetry_csv_ << report.waypoint_index << ',' << std::fixed
                   << std::setprecision(3) << report.target_position_ned_m.x()
                   << ',' << report.target_position_ned_m.y() << ','
                   << report.target_position_ned_m.z() << ','
                   << report.actual_position_ned_m.x() << ','
                   << report.actual_position_ned_m.y() << ','
                   << report.actual_position_ned_m.z() << ','
                   << report.position_error_ned_m.x() << ','
                   << report.position_error_ned_m.y() << ','
                   << report.position_error_ned_m.z() << ','
                   << report.position_error_norm_m << ','
                   << report.actual_velocity_ned_m_s.x() << ','
                   << report.actual_velocity_ned_m_s.y() << ','
                   << report.actual_velocity_ned_m_s.z() << ','
                   << report.actual_speed_norm_m_s << ','
                   << std::setprecision(2) << report.desired_duration_s << ','
                   << report.configured_max_horiz_speed_m_s << ','
                   << report.actual_duration_s << ',' << report.time_error_s
                   << '\n';
    telemetry_csv_.flush();  // 確保 Ctrl+C 中止模擬時不漏掉最後幾筆
  }

  return report;
}

void MyGotoMode::file_open() {
  // 1. 先建立資料夾（傳入 error_code 避免多執行緒同時建目錄拋出例外）
  namespace fs = std::filesystem;
  std::error_code ec;
  fs::create_directories("telemetry_logs", ec);

  // 2. 再開啟對應 UAV_ID 的檔案
  const std::string csv_path =
      "telemetry_logs/uav_" + std::to_string(UAV_ID) + ".csv";
  telemetry_csv_.open(csv_path, std::ios::out | std::ios::trunc);

  // 3. 確認開檔成功後寫入表頭
  if (telemetry_csv_.is_open()) {
    telemetry_csv_ << "wp_idx,"
                   << "target_x,target_y,target_z,"
                   << "actual_x,actual_y,actual_z,"
                   << "err_x,err_y,err_z,err_norm,"
                   << "vx,vy,vz,speed,"
                   << "expected_dt,target_speed,actual_dt,time_err\n";
  }
}