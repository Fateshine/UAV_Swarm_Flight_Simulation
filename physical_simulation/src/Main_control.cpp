#include "physical_simulation/Main_control.hpp"

MainControl::MainControl(const rclcpp::NodeOptions &options) : Node("MainControl", options)
{
  MemoryInit();
  Ros2MsgInit();
  ParmInit();
}

void MainControl::ParmInit()
{
  this->declare_parameter<int>("UAV_count");
  UAV_num_ = static_cast<uint16_t>(this->get_parameter("UAV_count").as_int());
}

void MainControl::MemoryInit()
{
  Action_clients_.reserve(UAV_num_ + 1);
  Trajectorys_all_UAV_flight_plan_.reserve(UAV_num_ + 1);
  Trajectorys_all_UAV_result_.reserve(UAV_num_ + 1);
  UAV_all_result_status_.reserve(UAV_num_ + 1);
  goal_handles_.reserve(UAV_num_ + 1);
}

void MainControl::Ros2MsgInit()
{
  callback_group_Reentrant_ = rclcpp::Node::create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  for (size_t i = 1; i <= UAV_num_; ++i)
  {
    std::string action_name = "/px4_" + std::to_string(i) + "/navigate";
    std::shared_ptr<rclcpp_action::Client<FlightAction>> action_client = rclcpp_action::create_client<FlightAction>(
        this,
        action_name,
        callback_group_Reentrant_,
        rcl_action_client_get_default_options());
    Action_clients_.push_back(action_client);
  }
}

void MainControl::GetFlightPlan(std::vector<nav_msgs::msg::Trajectory::ConstSharedPtr> flight_plan)
{
  Trajectorys_all_UAV_flight_plan_.clear();
  Trajectorys_all_UAV_result_.clear();
  UAV_all_result_status_.clear();

  size_t maxLen = 0;

  // 加上 const auto& 避免複製內層 vector
  for (const auto &row : flight_plan)
    maxLen = std::max(maxLen, row->points.size());

  Waypoint_num_ = maxLen;

  for (size_t i = 1; i <= UAV_num_; ++i)
  {
    Trajectorys_all_UAV_result_[i]->points.reserve(Waypoint_num_);
    UAV_all_result_status_[i]->waypoint_time_difference.reserve(Waypoint_num_);
    UAV_all_result_status_[i]->waypoint_distance_difference.reserve(Waypoint_num_);
  }

  for (size_t i = 1; i <= UAV_num_; ++i)
    Trajectorys_all_UAV_flight_plan_[i] = flight_plan[i];
  SendGoalAllUAV();
}

void MainControl::SendGoalAllUAV()
{
  RCLCPP_INFO(this->get_logger(), "=== 開始派發群飛任務 ===");
  for (size_t i = 1; i <= UAV_num_; ++i)
    SendGoal(i);
}

void MainControl::SendGoal(int16_t UAV_id)
{
  FlightAction::Goal flight_plan = FlightAction::Goal();

  flight_plan.trajectory = *Trajectorys_all_UAV_flight_plan_[UAV_id];
  rclcpp_action::Client<FlightAction>::SendGoalOptions send_goal_options = rclcpp_action::Client<FlightAction>::SendGoalOptions();

  send_goal_options.goal_response_callback = [this, UAV_id](const FlightGoalHandle::SharedPtr &goal_handle) -> void
  {
    return this->GoalResponseCallback(UAV_id, goal_handle);
  };

  send_goal_options.feedback_callback = [this, UAV_id](FlightGoalHandle::SharedPtr, const std::shared_ptr<const FlightAction::Feedback> feedback) -> void
  {
    return this->FeedbackCallback(UAV_id, feedback);
  };

  send_goal_options.result_callback = [this, UAV_id](const FlightGoalHandle::WrappedResult &result) -> void
  {
    return this->ResultCallback(UAV_id, result);
  };

  Action_clients_[UAV_id]->async_send_goal(flight_plan, send_goal_options);
}

void MainControl::GoalResponseCallback(int16_t UAV_id, const FlightGoalHandle::SharedPtr &goal_handle)
{
  if (!goal_handle)
    RCLCPP_ERROR(this->get_logger(), "UAV %d Goal was rejected by server", UAV_id);
  else
    RCLCPP_INFO(this->get_logger(), "UAV %d Goal accepted by server, waiting for result", UAV_id);
}

void MainControl::FeedbackCallback(int16_t UAV_id, const std::shared_ptr<const FlightAction::Feedback> feedback)
{
  Trajectorys_all_UAV_result_[UAV_id]->points.push_back(feedback->trajectorypoint);
}

void MainControl::ResultCallback(int16_t UAV_id, const FlightGoalHandle::WrappedResult &result)
{
  UAV_all_result_status_[UAV_id] = result.result;
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

void MainControl::CancelGoal(int16_t UAV_id)
{
  Action_clients_[UAV_id]->async_cancel_goal(
      goal_handles_[UAV_id],
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
