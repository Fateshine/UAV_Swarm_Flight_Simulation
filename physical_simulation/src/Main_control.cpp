#include "physical_simulation/Main_control.hpp"

MainControl::MainControl(const rclcpp::NodeOptions &options) : Node("MainControl", options)
{
  ParmInit();
  MemoryInit();
  Ros2MsgInit();
  // 以下為測試程式碼
  start_timer_ = this->create_wall_timer(
      std::chrono::seconds(10), std::bind(&MainControl::SendTestTrajectory, this));
}

void MainControl::ParmInit()
{
  rcl_interfaces::msg::ParameterDescriptor param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc.description = "The UAV's total number.";
  param_desc.read_only = true;
  this->declare_parameter<int>("UAV_count", param_desc);
  UAV_num_ = static_cast<uint16_t>(this->get_parameter("UAV_count").as_int());
}

void MainControl::MemoryInit()
{
  Action_clients_.resize(UAV_num_ + 1);
  Trajectorys_all_UAV_flight_plan_.resize(UAV_num_ + 1);
  Trajectorys_all_UAV_result_.resize(UAV_num_ + 1);
  UAV_all_result_status_.resize(UAV_num_ + 1);
  goal_handles_.resize(UAV_num_ + 1);
  for (size_t i = 1; i <= UAV_num_; ++i)
  {
    Trajectorys_all_UAV_result_[i] = std::make_shared<nav_msgs::msg::Trajectory>();
    UAV_all_result_status_[i] = std::make_shared<custom_interfaces::action::Flight_Result>();
  }
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
    Action_clients_[i] = action_client;
  }
}

void MainControl::GetFlightPlan(std::vector<nav_msgs::msg::Trajectory::ConstSharedPtr> flight_plan)
{
  // Trajectorys_all_UAV_flight_plan_.clear();
  // Trajectorys_all_UAV_result_.clear();
  // UAV_all_result_status_.clear();
  for (size_t i = 1; i < UAV_num_; ++i)
    Trajectorys_all_UAV_flight_plan_[i] = std::make_shared<nav_msgs::msg::Trajectory>();

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
    Trajectorys_all_UAV_flight_plan_[i] = flight_plan[i - 1];

  SendGoalAllUAV();

  Trajectorys_all_UAV_flight_plan_.clear();
}

void MainControl::SendGoalAllUAV()
{
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

  RCLCPP_INFO(this->get_logger(), "=== 開始派發UAV %d 群飛任務 ===", UAV_id);
}

void MainControl::GoalResponseCallback(int16_t UAV_id, const FlightGoalHandle::SharedPtr &goal_handle)
{
  if (!goal_handle)
    RCLCPP_ERROR(this->get_logger(), "UAV %d Goal was rejected by server", UAV_id);
  else
    RCLCPP_INFO(this->get_logger(), "UAV %d Goal accepted by server, waiting for result", UAV_id);

  goal_handles_[UAV_id] = goal_handle;
}

void MainControl::FeedbackCallback(int16_t UAV_id, const std::shared_ptr<const FlightAction::Feedback> feedback)
{
  Trajectorys_all_UAV_result_[UAV_id]->points.push_back(feedback->trajectorypoint);

  // RCLCPP_INFO(this->get_logger(), "Time %d.%09u UAV %d position %f %f %f, UAV velocity %f %f %f",
  //             feedback->trajectorypoint.header.stamp.sec,
  //             feedback->trajectorypoint.header.stamp.nanosec,
  //             UAV_id,
  //             feedback->trajectorypoint.pose.position.x,
  //             feedback->trajectorypoint.pose.position.y,
  //             feedback->trajectorypoint.pose.position.z,
  //             feedback->trajectorypoint.velocity.linear.x,
  //             feedback->trajectorypoint.velocity.linear.y,
  //             feedback->trajectorypoint.velocity.linear.z);
}

void MainControl::ResultCallback(int16_t UAV_id, const FlightGoalHandle::WrappedResult &result)
{
  UAV_all_result_status_[UAV_id] = result.result;

  RCLCPP_INFO(this->get_logger(), "UAV %d Result 平均距離誤差: %f 平均時間誤差: %f",
              UAV_id,
              UAV_all_result_status_[UAV_id]->waypoint_distance_difference_all,
              UAV_all_result_status_[UAV_id]->waypoint_time_difference_all);

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

void MainControl::SendTestTrajectory()
{
  // 1. 第一時間取消 Timer，確保只執行這一次
  if (start_timer_)
    start_timer_->cancel();

  std::vector<nav_msgs::msg::Trajectory::ConstSharedPtr> traj_ptr_all;
  traj_ptr_all.reserve(UAV_num_ + 1);
  for (size_t i = 1; i <= UAV_num_; i++)
  {
    nav_msgs::msg::Trajectory::ConstSharedPtr traj_ptr = TestTrajectory(this->get_clock()->now(), false);
    traj_ptr_all.push_back(traj_ptr);
  }
  GetFlightPlan(traj_ptr_all);
}

nav_msgs::msg::Trajectory::ConstSharedPtr MainControl::TestTrajectory(const rclcpp::Time &start_time,
                                                                      bool variable_speed,
                                                                      const std::string &frame_id,
                                                                      double altitude_z)
{
  auto traj = std::make_shared<nav_msgs::msg::Trajectory>();
  traj->header.stamp = start_time;
  traj->header.frame_id = frame_id;

  constexpr int kNumPoints = 31; // t = 0, 2, ..., 60 s
  constexpr double kDt = 2.0;    // 路徑點間隔 2 秒
  constexpr int kSubSteps = 100; // 積分細分步數 (0.02s/step)
  constexpr double kSubDt = kDt / kSubSteps;
  constexpr double kConstSpeed = 12.0; // 等速模式：12.0 m/s

  // 複合轉彎參數：主波 3 個週期 (6 次左右轉向) + 次諧波 1 個週期 (製造急彎與緩彎變化)
  constexpr double kYawAmp1 = 0.65; // 主偏航擺幅 (~37.2 deg)
  constexpr double kYawAmp2 = 0.25; // 次偏航擺幅 (~14.3 deg)

  // 總弧長：
  // - 等速模式：12.0 * 60 = 720.0 m
  // - 變速模式：80 + 110 + 150 + 160 + 130 + 75 = 705.0 m
  const double total_length = variable_speed ? 705.0 : 720.0;
  const double k_wave1 = 3.0 * (2.0 * M_PI / total_length);
  const double k_wave2 = 1.0 * (2.0 * M_PI / total_length);

  double x = 0.0;
  double y = 0.0;
  double s = 0.0;

  traj->points.reserve(kNumPoints);

  for (int i = 0; i < kNumPoints; ++i)
  {
    const double t = i * kDt;

    // 從上一個路徑點積分到當前時間 t (中點法數值積分)
    if (i > 0)
    {
      double t_sub = (i - 1) * kDt;
      for (int step = 0; step < kSubSteps; ++step)
      {
        const double v_mid = variable_speed
                                 ? getVariableSpeedProfile(t_sub + 0.5 * kSubDt).first
                                 : kConstSpeed;
        const double s_mid = s + 0.5 * v_mid * kSubDt;
        const double psi_mid = kYawAmp1 * std::sin(k_wave1 * s_mid) +
                               kYawAmp2 * std::sin(k_wave2 * s_mid);

        x += v_mid * std::cos(psi_mid) * kSubDt;
        y += v_mid * std::sin(psi_mid) * kSubDt;
        s += v_mid * kSubDt;
        t_sub += kSubDt;
      }
    }

    // 計算當前時間 t 的精確運動學狀態
    const auto [v, a_t] = variable_speed
                              ? getVariableSpeedProfile(t)
                              : std::make_pair(kConstSpeed, 0.0);

    const double psi = kYawAmp1 * std::sin(k_wave1 * s) +
                       kYawAmp2 * std::sin(k_wave2 * s);
    // 曲率 kappa = d(psi)/ds
    const double curvature = kYawAmp1 * k_wave1 * std::cos(k_wave1 * s) +
                             kYawAmp2 * k_wave2 * std::cos(k_wave2 * s);
    // 曲率變化率 d(kappa)/ds = d^2(psi)/ds^2
    const double d_curvature_ds = -kYawAmp1 * k_wave1 * k_wave1 * std::sin(k_wave1 * s) -
                                  kYawAmp2 * k_wave2 * k_wave2 * std::sin(k_wave2 * s);

    const double yaw_rate = v * curvature;
    const double yaw_accel = a_t * curvature + v * v * d_curvature_ds;
    const double a_normal = v * v * curvature; // 向心加速度

    nav_msgs::msg::TrajectoryPoint pt;
    pt.header.stamp = start_time + rclcpp::Duration::from_seconds(t);
    pt.header.frame_id = frame_id;

    // 1. Pose (ENU)
    pt.pose.position.x = x;
    pt.pose.position.y = y;
    pt.pose.position.z = altitude_z;
    pt.pose.orientation.x = 0.0;
    pt.pose.orientation.y = 0.0;
    pt.pose.orientation.z = std::sin(0.5 * psi);
    pt.pose.orientation.w = std::cos(0.5 * psi);

    // 2. Velocity (世界座標系 ENU)
    pt.velocity.linear.x = v * std::cos(psi);
    pt.velocity.linear.y = v * std::sin(psi);
    pt.velocity.linear.z = 0.0;
    pt.velocity.angular.x = 0.0;
    pt.velocity.angular.y = 0.0;
    pt.velocity.angular.z = yaw_rate;

    // 3. Acceleration (切線加速度 + 向心加速度)
    pt.acceleration.linear.x = a_t * std::cos(psi) - a_normal * std::sin(psi);
    pt.acceleration.linear.y = a_t * std::sin(psi) + a_normal * std::cos(psi);
    pt.acceleration.linear.z = 0.0;
    pt.acceleration.angular.x = 0.0;
    pt.acceleration.angular.y = 0.0;
    pt.acceleration.angular.z = yaw_accel;

    traj->points.push_back(std::move(pt));
  }

  return traj;
}

std::pair<double, double> MainControl::getVariableSpeedProfile(double t)
{
  if (t < 10.0)
  {
    return {8.0, 0.0}; // 0~10s: 等速 8.0 m/s (弧長 80m)
  }
  else if (t < 20.0)
  {
    constexpr double a_t = (14.0 - 8.0) / 10.0; // +0.60 m/s^2
    return {8.0 + a_t * (t - 10.0), a_t};       // 10~20s: 加速 8.0 -> 14.0 m/s (弧長 110m)
  }
  else if (t < 30.0)
  {
    constexpr double a_t = (16.0 - 14.0) / 10.0; // +0.20 m/s^2
    return {14.0 + a_t * (t - 20.0), a_t};       // 20~30s: 加速 14.0 -> 16.0 m/s (弧長 150m)
  }
  else if (t < 40.0)
  {
    return {16.0, 0.0}; // 30~40s: 極速巡航 16.0 m/s (弧長 160m)
  }
  else if (t < 50.0)
  {
    constexpr double a_t = (10.0 - 16.0) / 10.0; // -0.60 m/s^2
    return {16.0 + a_t * (t - 40.0), a_t};       // 40~50s: 減速 16.0 -> 10.0 m/s (弧長 130m)
  }
  else
  {
    constexpr double a_t = (5.0 - 10.0) / 10.0; // -0.50 m/s^2
    return {10.0 + a_t * (t - 50.0), a_t};      // 50~60s: 減速 10.0 -> 5.0 m/s (弧長 75m)
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(MainControl)
