#pragma once

#include <px4_ros2/components/mode_executor.hpp>

#include "physical_simulation/PX4_goto_mode.hpp"

// 模式執行器，負責管理高階狀態 (例如 起飛 -> 執行自訂 Mode -> 返航降落)
class MyGotoExecutor : public px4_ros2::ModeExecutorBase {
 public:
  // 定義外部 Callback 型態：當整個 Executor 結束時回傳 Result 與所有遙測報告
  using CompletedCallback = std::function<void(
      px4_ros2::Result, const std::vector<WaypointTelemetryReport> &)>;

  // 建構子接收自訂 Mode 實體，以及是否需要自動起飛與返航的旗標
  explicit MyGotoExecutor(MyGotoMode &owned_mode,
                          bool enable_takeoff_and_rtl = true);
  ~MyGotoExecutor() override = default;

  // 狀態機定義
  enum class State {
    Reset,              // 初始/重置狀態
    Arming,             // 先解鎖
    TakingOff,          // 自動起飛 (透過 PX4 內部指令)
    MyMode,             // 排程執行 MyGotoMode (會飛完所有 Waypoints)
    RTL,                // 返航模式
    WaitUntilDisarmed,  // 等待螺旋槳停止
  };

  /**
   * @brief onActivate() function override, ModeExecutorBase
   * 必須覆寫的生命週期函數
   *
   */
  void onActivate() override;

  /**
   * @brief onDeactivate function override, ModeExecutorBase
   * 必須覆寫的生命週期函數
   *
   * @param reason 被停用的原因
   */
  void onDeactivate(DeactivateReason reason) override;

  /**
   * @brief 執行各狀態轉換
   *
   * @param state 狀態
   * @param previous_result 上個狀態結果
   */
  void runState(State state, px4_ros2::Result previous_result);

  // ================= 供外部 Composable Node (Action Server) 操作的 API
  // =================

  /**
   * @brief 設定航點任務，會轉交給內部的 MyGotoMode
   *
   * @param waypoints 飛行計畫
   */
  void setWaypoints(const std::vector<WaypointTask> &waypoints);

  /**
   * @brief 用來接 Action server 的 callback function
   *
   * @param callback Action server 的 callback function
   */
  void setOnCompletedCallback(CompletedCallback callback);

  // /**
  //  * @brief Get the all Telemetry Reports object
  //  *
  //  * @return const std::vector<WaypointTelemetryReport>&
  //  */
  // const std::vector<WaypointTelemetryReport> &getTelemetryReports() const;

  /**
   * @brief Get the Current Position Ned of UAV
   *
   * @return Eigen::Vector3f
   */
  Eigen::Vector3f getCurrentPositionNed() const;
  /**
   * @brief Get the Current Velocity Ned of UAV
   *
   * @return Eigen::Vector3f
   */
  Eigen::Vector3f getCurrentVelocityNed() const;

 private:
  // 來自 owned_mode.node()，用於 Log 輸出
  rclcpp::Node &_node;
  // 持有 MyGotoMode 參考
  MyGotoMode &_custom_mode;
  // 是否執行 Takeoff 與 RTL
  bool _enable_takeoff_and_rtl{true};
  // 用來接Action server的 callback function
  CompletedCallback _on_completed_callback{nullptr};
};