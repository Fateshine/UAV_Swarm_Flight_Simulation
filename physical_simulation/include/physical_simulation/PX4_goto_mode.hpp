#pragma once

#include <Eigen/Core>
#include <filesystem>
#include <fstream>
#include <optional>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

using namespace px4_ros2::literals;  // NOLINT

// 定義單一路徑點輸入結構，供外部 Action Server 傳入
struct WaypointTask {
  Eigen::Vector3f
      target_offset_ned_m;   // 相對於模式啟動點的 NED 偏移座標 [北, 東, 下] (m)
  float desired_duration_s;  // 期望抵達此路徑點所花費的時間 (s)
  bool face_direction_of_travel{true};  // 是否讓機頭自動朝向飛行方向
};

// 單一路徑點抵達時的遙測與誤差結算報告 (供外部 Node 讀取，方便數據蒐集)
struct WaypointTelemetryReport {
  size_t waypoint_index{0};
  Eigen::Vector3f target_position_ned_m{0.f, 0.f, 0.f};    // 指定目標
  Eigen::Vector3f actual_position_ned_m{0.f, 0.f, 0.f};    // 實際抵達位置
  Eigen::Vector3f position_error_ned_m{0.f, 0.f, 0.f};     // 三軸位置誤差
  float position_error_norm_m{0.f};                        // 總歐氏距離誤差
  Eigen::Vector3f actual_velocity_ned_m_s{0.f, 0.f, 0.f};  // 抵達瞬間速度向量
  float actual_speed_norm_m_s{0.f};                        // 抵達瞬間合速度
  float actual_yaw_deg{0.f};                  // 抵達瞬間機頭朝向 (度)
  float configured_max_horiz_speed_m_s{0.f};  // 換算出的水平限速
  double desired_duration_s{0.0};             // 期望耗時
  double actual_duration_s{0.0};              // 實際耗時
  double time_error_s{0.0};                   // 時間誤差 (正值代表延遲)
};

/**
 * @class MyGotoMode
 * @brief 自訂飛行模式，讓無人機朝向指定連續軌跡點飛行
 * 注意：本類別並非獨立的 rclcpp::Node，而是掛載於外部 Component 之下的邏輯實體
 */
class MyGotoMode : public px4_ros2::ModeBase {
 public:
  // 建構子強制接收外部 rclcpp::Node 參考，以建立與 PX4 溝通的底層
  // Publisher/Subscriber
  explicit MyGotoMode(rclcpp::Node &node);
  ~MyGotoMode() override = default;

  /**
   * @brief onActivate() function override, ModeBase 必須覆寫的生命週期函數
   *
   */
  void onActivate() override;

  /**
   * @brief onDeactivate function override, ModeBase 必須覆寫的生命週期函數
   *
   */
  void onDeactivate() override;

  /**
   * @brief 由 PX4 內部高頻 Timer 呼叫 (發送 Setpoint 邏輯寫在這)
   *
   * @param dt_s 距離上一次呼叫 updateSetpoint 所經過的時間間隔
   */
  void updateSetpoint(float dt_s)
      override;  // 由 PX4 內部高頻 Timer 呼叫 (發送 Setpoint 邏輯寫在這)

  // ================= 供 MyGotoExecutor (及外部 Component) 呼叫的 API
  // =================
  /**
   * @brief 設定航點任務
   *
   * @param waypoints 飛行計畫
   */
  void setWaypoints(const std::vector<WaypointTask> &waypoints);

  /**
   * @brief 取得已完成航點的遙測結算數據
   *
   * @return const std::vector<WaypointTelemetryReport>&
   */
  const std::vector<WaypointTelemetryReport> &getTelemetryReports() const;

  /**
   * @brief Get the Current Position Ned of UAV  (透過
   * px4_ros2::OdometryLocalPosition)
   *
   * @return Eigen::Vector3f
   */
  Eigen::Vector3f getCurrentPositionNed() const;

  /**
   * @brief Get the Current Velocity Ned of UAV  (透過
   * px4_ros2::OdometryLocalPosition)
   *
   * @return Eigen::Vector3f
   */
  Eigen::Vector3f getCurrentVelocityNed() const;

 private:
  // (測試用)寫入無人機telemetry資料
  std::ofstream telemetry_csv_;
  uint16_t UAV_ID;
  // 狀態機：模式啟動後先穩定懸停，再開始執行路徑點
  enum class State {
    SettlingAtStart = 0,
    ExecutingWaypoints
  } _state{State::SettlingAtStart};

  /**
   * @brief 初始化新航段並計算速度
   *
   * @param target_position_m 目標位置
   */
  void initCurrentSegment(const Eigen::Vector3f &target_position_m);

  /**
   * @brief 檢查位置誤差是否進入允差半徑
   *
   * @param target_position_m 目標位置
   * @return true 是
   * @return false 否
   */
  bool positionReached(const Eigen::Vector3f &target_position_m) const;

  /**
   * @brief 計算目標航向與當前機頭朝向（Yaw）的誤差，如果誤差小於 7
   * 度，則判定為「機頭已轉到指定方向」
   *
   * @param target_heading_rad 目標航向
   * @return true 是
   * @return false 否
   */
  bool headingReached(float target_heading_rad) const;

  /**
   * @brief 填寫並報告當前航點telemony數據
   *
   * @param wp_index 航點index
   * @param target_position_m 目標位置
   * @return WaypointTelemetryReport 該航點telemony數據
   */
  WaypointTelemetryReport recordAndPrintReport(
      size_t wp_index, const Eigen::Vector3f &target_position_m);

  /**
   * @brief (測試用) 開啟要寫入的telemony數據檔案
   *
   */
  void file_open();

  //  ================= PX4 官方介面：負責發送 Goto 指令與接收遙測
  //  =================
  // 用來發送「飛往特定全域座標 (Global Position)」的指令。
  std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
  // 用來讀取無人機當前的經緯度與高度。
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
  // 用來讀取無人機當前的姿態（包含機頭朝向 Yaw）。
  std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
  // =============================================================================

  // 記錄模式剛啟動時的起始位置
  Eigen::Vector3f _start_position_m{0.f, 0.f, 0.f};
  bool _start_position_set{false};

  // 航段執行狀態紀錄
  std::vector<WaypointTask> _waypoints;
  std::vector<WaypointTelemetryReport> _reports;
  size_t _current_wp_index{0};
  bool _segment_initialized{false};

  Eigen::Vector3f _segment_start_pos_m{0.f, 0.f, 0.f};
  rclcpp::Time _segment_start_time{};  // 與外部 Node 共用 ROS Clock

  // 透過距離/時間換算出來的動態速度限制 (餵給 Goto 控制器)
  float _max_horizontal_velocity_m_s{3.0f};
  float _max_vertical_velocity_m_s{1.5f};

  // 物理閾值與補償常數
  static constexpr float kPositionErrorThreshold =
      1.5f;  // 進入此半徑視為「抵達」 [m]
  static constexpr float kHeadingErrorThreshold =
      7.0_deg;  // 誤差為7deg以下即視為成功朝向
  static constexpr float kRampCompensation =
      1.20f;  // 補償起步加速與抵達前減速耗損的時間
};