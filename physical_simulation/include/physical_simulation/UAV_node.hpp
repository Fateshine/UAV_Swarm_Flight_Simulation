#pragma once

#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <px4_ros2/components/wait_for_fmu.hpp>

#include "custom_interfaces/action/flight.hpp"
#include "physical_simulation/PX4_executor.hpp"

class UAVNode final : public rclcpp::Node
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
    virtual ~UAVNode() override = default;

private:
    // 無人機的 Action server
    rclcpp_action::Server<FlightAction>::SharedPtr Action_servers_;
    // 無人機的 Action client 號碼牌
    std::shared_ptr<FlightGoalHandle> Current_goal_handle_;
    // 無人機ID
    uint16_t UAV_ID_;
    // 單次飛行計畫會有的路徑點數量
    uint16_t Waypoint_num_;
    // 無人機的飛行計畫 (格式詳情參閱 https://docs.ros.org/en/jazzy/p/nav_msgs/msg/Trajectory.html)
    nav_msgs::msg::Trajectory::ConstSharedPtr Trajectorys_UAV_flight_plan_;
    // 無人機的模擬飛行結果 (格式詳情參閱 https://docs.ros.org/en/jazzy/p/nav_msgs/msg/Trajectory.html)
    nav_msgs::msg::Trajectory::SharedPtr Trajectorys_UAV_result_;
    // 當前的無人機狀態 (格式詳情參閱 https://docs.ros.org/en/jazzy/p/nav_msgs/msg/TrajectoryPoint.html)
    FlightAction::Feedback::SharedPtr UAV_feedback_;
    // 無人機的飛行結果狀態 (格式詳情參閱 ros2_ws/src/custom_interfaces/action/Flight.action)
    FlightAction::Result::SharedPtr UAV_result_status_;

    // 回調函數群組
    rclcpp::CallbackGroup::SharedPtr callback_group_Reentrant_;
    // 用來在背景定時發送 Feedback
    rclcpp::TimerBase::SharedPtr _feedback_timer;

    // 內部自訂義 PX4 class
    std::unique_ptr<MyGotoMode> _goto_mode;
    // 內部自訂義 PX4 class
    std::unique_ptr<MyGotoExecutor> _goto_executor;

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
     * @brief 處理目標訊息
     *
     * @param uuid 辨識目標
     * @param goal 目標
     * @return rclcpp_action::GoalResponse
     */
    rclcpp_action::GoalResponse HandleGoal(const rclcpp_action::GoalUUID &uuid, FlightAction::Goal::ConstSharedPtr goal);

    /**
     * @brief 處理目標取消
     *
     * @param goal_handle 用於追蹤、控制和更新一個特定 Action Goal 的生命週期狀態
     * @return rclcpp_action::CancelResponse
     */
    rclcpp_action::CancelResponse HandleCancel(const std::shared_ptr<FlightGoalHandle> goal_handle);

    /**
     * @brief 確認目標後執行
     *
     * @param goal_handle 用於追蹤、控制和更新一個特定 Action Goal 的生命週期狀態
     */
    void HandleAccepted(const std::shared_ptr<FlightGoalHandle> goal_handle);

    /**
     * @brief 實際目標執行函式
     *
     * @param goal_handle 用於追蹤、控制和更新一個特定 Action Goal 的生命週期狀態
     */
    void ExecuteFlight(const std::shared_ptr<FlightGoalHandle> goal_handle);
};
