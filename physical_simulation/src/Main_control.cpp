#include "physical_simulation/Main_control.hpp"

MainControl::MainControl(const rclcpp::NodeOptions &options) : Node("MainControl", options)
{
  memory_init();
  ros2_msg_init();
}

void MainControl::memory_init()
{
  Action_clients.reserve(UAV_num);
  Trajectorys_all_UAV_flight_plan.reserve(UAV_num);
  Trajectorys_all_UAV_result.reserve(UAV_num);
  UAV_all_result_status.reserve(UAV_num);

  for (size_t i = 0; i < UAV_num; ++i)
  {
    UAV_all_result_status[i]->waypoint_time_difference.reserve(Waypoint_num);
    UAV_all_result_status[i]->waypoint_distance_difference.reserve(Waypoint_num);
  }
}

void MainControl::ros2_msg_init()
{
  callback_group_Reentrant = rclcpp::Node::create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  for (size_t i = 0; i < UAV_num; ++i)
  {
    std::string action_name = "/" + std::to_string(i) + "/navigate";
    std::shared_ptr<rclcpp_action::Client<FlightAction>> action_client = rclcpp_action::create_client<FlightAction>(
        this,
        action_name,
        callback_group_Reentrant,
        rcl_action_client_get_default_options());
    Action_clients.push_back(action_client);
  }
}

void MainControl::get_flight_plan(std::vector<nav_msgs::msg::Trajectory::ConstSharedPtr> flight_plan)
{
  Trajectorys_all_UAV_flight_plan.clear();
  Trajectorys_all_UAV_result.clear();
  UAV_all_result_status.clear();

  for (size_t i = 0; i < UAV_num; ++i)
    Trajectorys_all_UAV_flight_plan[i] = flight_plan[i];
  send_goal_all_UAV();
}

void MainControl::send_goal_all_UAV()
{
  RCLCPP_INFO(this->get_logger(), "=== 開始派發群飛任務 ===");
  for (size_t i = 0; i < UAV_num; ++i)
    send_goal(i);
}

void MainControl::send_goal(int16_t UAV_id)
{
  FlightAction::Goal flight_plan = FlightAction::Goal();

  flight_plan.trajectory = std::move(*Trajectorys_all_UAV_flight_plan[UAV_id]);
  rclcpp_action::Client<FlightAction>::SendGoalOptions send_goal_options = rclcpp_action::Client<FlightAction>::SendGoalOptions();

  send_goal_options.goal_response_callback = [this, UAV_id](const FlightGoalHandle::SharedPtr &goal_handle) -> void
  {
    return this->goal_response_callback(UAV_id, goal_handle);
  };

  send_goal_options.feedback_callback = [this, UAV_id](FlightGoalHandle::SharedPtr, const std::shared_ptr<const FlightAction::Feedback> feedback) -> void
  {
    return this->feedback_callback(UAV_id, feedback);
  };

  send_goal_options.result_callback = [this, UAV_id](const FlightGoalHandle::WrappedResult &result) -> void
  {
    return this->result_callback(UAV_id, result);
  };

  Action_clients[UAV_id]->async_send_goal(flight_plan, send_goal_options);
}

void MainControl::goal_response_callback(int16_t UAV_id, const FlightGoalHandle::SharedPtr &goal_handle)
{
  if (!goal_handle)
    RCLCPP_ERROR(this->get_logger(), "UAV %d Goal was rejected by server", UAV_id);
  else
    RCLCPP_INFO(this->get_logger(), "UAV %d Goal accepted by server, waiting for result", UAV_id);
}

void MainControl::feedback_callback(int16_t UAV_id, const std::shared_ptr<const FlightAction::Feedback> feedback)
{
  Trajectorys_all_UAV_result[UAV_id]->points.push_back(feedback->trajectorypoint);
}

void MainControl::result_callback(int16_t UAV_id, const FlightGoalHandle::WrappedResult &result)
{
  UAV_all_result_status[UAV_id] = result.result;
  switch (result.code)
  {
  case rclcpp_action::ResultCode::SUCCEEDED:
    RCLCPP_INFO(this->get_logger(), "[無人機 %d] 任務成功完成", UAV_id);
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(this->get_logger(), "[無人機 %d] 任務異常中止(失敗)", UAV_id);
    return;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_WARN(this->get_logger(), "[無人機 %d] 任務被取消", UAV_id);
    return;
  case rclcpp_action::ResultCode::UNKNOWN:
  default:
    RCLCPP_ERROR(this->get_logger(), "[無人機 %d] 未知狀態", UAV_id);
    return;
  }
}

void MainControl::cancel_goal(int16_t UAV_id)
{
  Action_clients[UAV_id]->async_cancel_goal(
      goal_handles[UAV_id],
      [this, UAV_id](action_msgs::srv::CancelGoal_Response::SharedPtr cancel_response)
      {
        switch (cancel_response->return_code)
        {
        case action_msgs::srv::CancelGoal_Response::ERROR_NONE:
          RCLCPP_INFO(this->get_logger(), "UAV %d  成功受理飛行計畫取消請求！", UAV_id);
          break;
        case action_msgs::srv::CancelGoal_Response::ERROR_REJECTED:
          RCLCPP_ERROR(this->get_logger(), "UAV %d 取消請求失敗！取消請求被 Server 拒絕", UAV_id);
          return;
        case action_msgs::srv::CancelGoal_Response::ERROR_UNKNOWN_GOAL_ID:
          RCLCPP_ERROR(this->get_logger(), "UAV %d 取消請求失敗！找不到這個任務", UAV_id);
          return;
        case action_msgs::srv::CancelGoal_Response::ERROR_GOAL_TERMINATED:
          RCLCPP_ERROR(this->get_logger(), "UAV %d 取消請求失敗！任務已經終止", UAV_id);
          return;
        }
      });
}

RCLCPP_COMPONENTS_REGISTER_NODE(MainControl)
