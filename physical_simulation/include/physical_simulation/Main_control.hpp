#pragma once

#include <string>
#include <vector>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "custom_interfaces/action/flight.hpp"

class MainControl : public rclcpp::Node
{
public:
  using FlightAction = custom_interfaces::action::Flight;
  using FlightGoalHandle = rclcpp_action::ClientGoalHandle<FlightAction>;

  /**
   * @brief 初始化
   *
   * @param options Node設定細節(Component,Zero copy)
   */
  explicit MainControl(const rclcpp::NodeOptions &options);

private:
  // 所有無人機的 Action client
  std::vector<rclcpp_action::Client<FlightAction>::SharedPtr> Action_clients;
  // 所有無人機的 Action server 號碼牌
  std::vector<FlightGoalHandle::SharedPtr> goal_handles;
  // 無人機數量
  uint16_t UAV_num;
  // 單次飛行計畫會有的路徑點數量
  uint16_t Waypoint_num;
  // 所有無人機的飛行計畫 (格式詳情參閱 https://docs.ros.org/en/jazzy/p/nav_msgs/msg/Trajectory.html)
  std::vector<nav_msgs::msg::Trajectory::ConstSharedPtr> Trajectorys_all_UAV_flight_plan;
  // 所有無人機的模擬飛行結果 (格式詳情參閱 https://docs.ros.org/en/jazzy/p/nav_msgs/msg/Trajectory.html)
  std::vector<nav_msgs::msg::Trajectory::SharedPtr> Trajectorys_all_UAV_result;
  // 所有無人機的飛行結果狀態 (格式詳情參閱 ros2_ws/src/custom_interfaces/action/Flight.action)
  std::vector<FlightAction::Result::SharedPtr> UAV_all_result_status;

  // 回調函數群組
  rclcpp::CallbackGroup::SharedPtr callback_group_Reentrant;

  /**
   * @brief 預先配置記憶體，避免動態擴容造成記憶體碎片化
   *
   */
  void memory_init();

  /**
   * @brief 初始化所有Ros2訊息
   *
   */
  void ros2_msg_init();

  /**
   * @brief 獲得飛行計畫
   *
   * @param flight_plan 飛行計畫
   */
  void get_flight_plan(std::vector<nav_msgs::msg::Trajectory::ConstSharedPtr> flight_plan);

  /**
   * @brief 派發飛行計畫給所有無人機
   *
   */
  void send_goal_all_UAV();

  /**
   * @brief 派發飛行計畫給單ㄧ無人機
   *
   * @param UAV_id 無人機ID
   */

  void send_goal(int16_t UAV_id);
  /**
   * @brief 處理傳達Goal之後收到的回覆
   *
   * @param UAV_id 無人機ID
   * @param goal_handle 號碼牌
   */
  void goal_response_callback(int16_t UAV_id, const FlightGoalHandle::SharedPtr &goal_handle);

  /**
   * @brief 處理feedback回覆
   *
   * @param UAV_id  無人機ID
   * @param feedback feedback訊息
   */
  void feedback_callback(int16_t UAV_id, const std::shared_ptr<const FlightAction::Feedback> feedback);

  /**
   * @brief 處理result結果訊息
   *
   * @param UAV_id 無人機ID
   * @param result result訊息
   */
  void result_callback(int16_t UAV_id, const FlightGoalHandle::WrappedResult &result);

  /**
   * @brief 處理取消請求
   *
   * @param UAV_id 無人機ID
   */
  void cancel_goal(int16_t UAV_id);
};
