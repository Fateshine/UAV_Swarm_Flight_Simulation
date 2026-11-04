#include "physical_simulation/UAV_node.hpp"

UAVNode::UAVNode(const rclcpp::NodeOptions &options) : Node("UAVNode", options)
{
    memory_init();
    ros2_msg_init();
}

void UAVNode::memory_init()
{
    UAV_feedback = std::make_shared<FlightAction::Feedback>();
    UAV_result_status = std::make_shared<FlightAction::Result>();
}

void UAVNode::ros2_msg_init()
{
    callback_group_Reentrant = rclcpp::Node::create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    std::string action_name = "/" + std::to_string(UAV_ID) + "/navigate";
    Action_servers = rclcpp_action::create_server<FlightAction>(
        this,
        action_name,
        // (1) handle_goal: 接收 uuid 和 goal_msg
        [this](const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FlightAction::Goal> goal) -> rclcpp_action::GoalResponse
        {
            return this->handle_goal(uuid, goal);
        },

        // (2) handle_cancel: 接收 goal_handle
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<FlightAction>> goal_handle) -> rclcpp_action::CancelResponse
        {
            return this->handle_cancel(goal_handle);
        },

        // (3) handle_accepted: 接收 goal_handle
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<FlightAction>> goal_handle) -> void
        {
            this->handle_accepted(goal_handle);
        },
        rcl_action_server_get_default_options(),
        callback_group_Reentrant);
}

rclcpp_action::GoalResponse UAVNode::handle_goal(const rclcpp_action::GoalUUID &uuid, FlightAction::Goal::ConstSharedPtr goal)
{
    RCLCPP_INFO(this->get_logger(), "UAV %d Received goal request", UAV_ID);
    (void)uuid;

    // 檢查軌跡是否為空
    if (goal->trajectory.points.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "[UAV %d] 軌跡為空，拒絕任務", UAV_ID);
        return rclcpp_action::GoalResponse::REJECT;
    }

    Trajectorys_UAV_flight_plan = std::shared_ptr<const nav_msgs::msg::Trajectory>(goal, &(goal->trajectory));
    Waypoint_num = goal->trajectory.points.size();

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse UAVNode::handle_cancel(const std::shared_ptr<FlightGoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "[UAV %d] 收到取消任務請求", UAV_ID);
    (void)goal_handle;

    return rclcpp_action::CancelResponse::ACCEPT;
}

void UAVNode::handle_accepted(const std::shared_ptr<FlightGoalHandle> goal_handle)
{
    // 更新當前控制的 handle，用於 cancel 判斷
    Current_goal_handle = goal_handle;

    // 開啟一條背景 Thread 執行實際的迴圈，避免卡死 ROS 2 Spin 迴圈
    // std::thread{std::bind(&UAVNode::execute_flight, this, std::placeholders::_1), goal_handle}.detach();
}

void UAVNode::execute_flight(const std::shared_ptr<FlightGoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "[UAV %d] 開始執行飛行任務，總航點數: %d", UAV_ID, Waypoint_num);

    rclcpp::Rate loop_rate(5); // 假設控制迴圈為 5Hz，請依實際情況調整

    // 取得內部 points 陣列的參考
    const std::vector<nav_msgs::msg::TrajectoryPoint> &Trajectory_points_plan = Trajectorys_UAV_flight_plan->points;

    // 開始走訪軌跡
    for (int i = 0; i < Waypoint_num; ++i)
    {
        // 檢查 Client 是否發送了取消請求
        if (goal_handle->is_canceling())
        {
            // 如果成立先將結果寫回result
            goal_handle->canceled(UAV_result_status);
            return;
        }

        // ============================================
        // 這裡放入底層物理控制或 Gazebo 控制指令
        // 讀取 Trajectory_points_plan[i] 並透過 Publisher 發給 PX4
        // 然後Trajectorys_UAV_result->points[i]獲得模擬器資料
        // ============================================

        // 準備發布 Feedback
        UAV_feedback->trajectorypoint = Trajectorys_UAV_result->points[i];
        goal_handle->publish_feedback(UAV_feedback);

        // 記錄此點的誤差 (範例假資料)
        UAV_result_status->waypoint_time_difference.push_back(0.01 * i);
        UAV_result_status->waypoint_distance_difference.push_back(0.02 * i);

        // ============================================
        // 這裡判斷目標邏輯
        // 例如發生碰撞(任務中止 / 失敗) goal_handle->abort(UAV_result_status);
        // 例如抵達最後目標(任務成功) goal_handle->succeed(UAV_result_status);
        // 如果以上成立先將結果寫回result
        // ============================================

        // 控制迴圈休眠
        loop_rate.sleep();
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(UAVNode)