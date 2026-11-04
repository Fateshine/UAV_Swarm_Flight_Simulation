#pragma once

#include <string>
#include <vector>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "custom_interfaces/action/flight.hpp"

class UAVNode : public rclcpp::Node
{
public:
    using FlightAction = custom_interfaces::action::Flight;
    using FlightGoalHandle = rclcpp_action::ServerGoalHandle<FlightAction>;

    /**
     * @brief 初始化
     *
     * @param options Node設定細節(Component,Zero copy)
     */
    explicit UAVNode(const rclcpp::NodeOptions &options);

private:
    // 無人機的 Action server
    rclcpp_action::Server<FlightAction>::SharedPtr Action_servers;
    // 無人機的 Action client 號碼牌
    std::shared_ptr<FlightGoalHandle> Current_goal_handle;
    // 無人機ID
    uint16_t UAV_ID;
    // 單次飛行計畫會有的路徑點數量
    uint16_t Waypoint_num;
    // 無人機的飛行計畫 (格式詳情參閱 https://docs.ros.org/en/jazzy/p/nav_msgs/msg/Trajectory.html)
    nav_msgs::msg::Trajectory::ConstSharedPtr Trajectorys_UAV_flight_plan;
    // 無人機的模擬飛行結果 (格式詳情參閱 https://docs.ros.org/en/jazzy/p/nav_msgs/msg/Trajectory.html)
    nav_msgs::msg::Trajectory::SharedPtr Trajectorys_UAV_result;
    // 當前的無人機狀態 (格式詳情參閱 https://docs.ros.org/en/jazzy/p/nav_msgs/msg/TrajectoryPoint.html)
    FlightAction::Feedback::SharedPtr UAV_feedback;
    // 無人機的飛行結果狀態 (格式詳情參閱 ros2_ws/src/custom_interfaces/action/Flight.action)
    FlightAction::Result::SharedPtr UAV_result_status;

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

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, FlightAction::Goal::ConstSharedPtr goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<FlightGoalHandle> goal_handle);
    void handle_accepted(const std::shared_ptr<FlightGoalHandle> goal_handle);
    void execute_flight(const std::shared_ptr<FlightGoalHandle> goal_handle);
};
