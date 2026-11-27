#pragma once

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <string>
#include <unordered_set>
#include <vector>

#include "custom_interfaces/action/flight.hpp"
#include "physical_simulation/Gazebo_control.hpp"

/**
 * @class MainControl
 * @brief 用於控制主模擬系統 (包含各自無人機和Gazebo)
 */
class MainControl final : public rclcpp::Node {
 public:
  using FlightAction = custom_interfaces::action::Flight;
  using FlightGoalHandle = rclcpp_action::ClientGoalHandle<FlightAction>;
  using SimState = GazeboSimController::SimulationState;

  /**
   * @brief 初始化
   *
   * @param options Node設定細節(Component,Zero copy)
   */
  explicit MainControl(const rclcpp::NodeOptions &options);
  virtual ~MainControl() override = default;

 private:
  // 無人機數量
  uint16_t UAV_num_;
  // 單次飛行計畫會有的路徑點數量
  uint16_t Waypoint_num_;
  // 確定每一台無人機是否準備完成
  std::unordered_set<uint16_t> ready_uavs_;
  // 所有無人機的飛行計畫 (格式詳情參閱
  // https://docs.ros.org/en/jazzy/p/nav_msgs/msg/Trajectory.html)
  std::vector<nav_msgs::msg::Trajectory::ConstSharedPtr>
      Trajectorys_all_UAV_flight_plan_;
  // 所有無人機的模擬飛行結果 (格式詳情參閱
  // https://docs.ros.org/en/jazzy/p/nav_msgs/msg/Trajectory.html)
  std::vector<nav_msgs::msg::Trajectory::SharedPtr> Trajectorys_all_UAV_result_;
  // 所有無人機的飛行結果狀態 (格式詳情參閱
  // ros2_ws/src/custom_interfaces/action/Flight.action)
  std::vector<FlightAction::Result::SharedPtr> UAV_all_result_status_;

  // 所有無人機的 Action client
  std::vector<rclcpp_action::Client<FlightAction>::SharedPtr> Action_clients_;
  // 所有無人機的 Action server 號碼牌
  std::vector<FlightGoalHandle::SharedPtr> goal_handles_;
  // 確保所有無人機準備好的 Subscription
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr ready_sub_;
  // 回調函數群組
  rclcpp::CallbackGroup::SharedPtr callback_group_Reentrant_;
  rclcpp::CallbackGroup::SharedPtr callback_group_MutuallyExclusive_,
      callback_group_MutuallyExclusive_Gz;
  // 確保所有無人機準備好的timer
  rclcpp::TimerBase::SharedPtr ready_timer_;
  // 內部自訂義 Gazebo control class
  std::unique_ptr<GazeboSimController> Gazebo_control_;

  /**
   * @brief 接收所有參數
   *
   */
  void ParmInit();

  /**
   * @brief 預先配置記憶體，避免動態擴容造成記憶體碎片化
   *
   */
  void MemoryInit();

  /**
   * @brief 初始化所有Ros2訊息
   *
   */
  void Ros2MsgInit();

  /**
   * @brief 初始化所有Gazebo訊息
   *
   */
  void GzMsgInit();

  /**
   * @brief 確認所有無人機準備就緒
   *
   */
  void checkReadyBarrier();

  /**
   * @brief 獲得飛行計畫
   *
   * @param flight_plan 飛行計畫
   */
  void GetFlightPlan(
      std::vector<nav_msgs::msg::Trajectory::ConstSharedPtr> flight_plan);

  /**
   * @brief 派發飛行計畫給所有無人機
   *
   */
  void SendGoalAllUAV();

  /**
   * @brief 派發飛行計畫給單ㄧ無人機
   *
   * @param UAV_id 無人機ID
   */
  void SendGoal(int16_t UAV_id, rclcpp::Time start_time);

  /**
   * @brief 處理傳達Goal之後收到的回覆
   *
   * @param UAV_id 無人機ID
   * @param goal_handle 號碼牌
   */
  void GoalResponseCallback(int16_t UAV_id,
                            const FlightGoalHandle::SharedPtr &goal_handle);

  /**
   * @brief 處理feedback回覆
   *
   * @param UAV_id  無人機ID
   * @param feedback feedback訊息
   */
  void FeedbackCallback(
      int16_t UAV_id,
      const std::shared_ptr<const FlightAction::Feedback> feedback);

  /**
   * @brief 處理result結果訊息
   *
   * @param UAV_id 無人機ID
   * @param result result訊息
   */
  void ResultCallback(int16_t UAV_id,
                      const FlightGoalHandle::WrappedResult &result);

  /**
   * @brief 處理取消請求
   *
   * @param UAV_id 無人機ID
   */
  void CancelGoal(int16_t UAV_id);

  /**
   * @brief 暫停模擬
   *
   */
  void PauseSimulation();

  /**
   * @brief 繼續模擬
   *
   */
  void ResumeSimulation();

  /**
   * @brief 獲取模擬狀態
   *
   */
  void GetSimulationState(GazeboSimController::StateCallback on_result);

  /**
   * @brief (測試用)發送軌跡
   *
   */
  void SendTestTrajectory();

  /**
   * @brief (測試用)軌跡產生器
   *
   */
  nav_msgs::msg::Trajectory::ConstSharedPtr TestTrajectory(
      const rclcpp::Time &start_time, bool variable_speed,
      const std::string &frame_id = "map", double altitude_z = 50.0);

  /**
   * @brief (測試用)計算軌跡 B (5 ~ 25 m/s，每 10 秒切換一次速度狀態) 在時間 t
   * 的 (速度 v, 切線加速度 a_t)
   *
   * @param t 時間
   * @return std::pair<double, double> 速度 v, 切線加速度 a_t
   */
  std::pair<double, double> getVariableSpeedProfile(double t);
};
