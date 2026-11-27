#include "physical_simulation/UAV_node.hpp"

UAVNode::UAVNode(const rclcpp::NodeOptions &options)
    : Node("UAVNode", options) {
  ParmInit();
  MemoryInit();
  Ros2MsgInit();
  initPX4Connection();
}

void UAVNode::ParmInit() {
  rcl_interfaces::msg::ParameterDescriptor param_desc =
      rcl_interfaces::msg::ParameterDescriptor{};
  param_desc.description =
      "The UAV's unique identifier starts with 1 to match the flight control "
      "system.";
  param_desc.read_only = true;
  this->declare_parameter<int>("UAV_id", param_desc);
  UAV_ID_ = static_cast<uint16_t>(this->get_parameter("UAV_id").as_int());
}

void UAVNode::MemoryInit() {
  UAV_feedback_ = std::make_shared<FlightAction::Feedback>();
  UAV_result_status_ = std::make_shared<FlightAction::Result>();

  // 建立 MyGotoMode (注入 *this) 與 MyGotoExecutor
  _goto_mode = std::make_unique<MyGotoMode>(*this);
  _goto_executor = std::make_unique<MyGotoExecutor>(*_goto_mode, true);
}

void UAVNode::Ros2MsgInit() {
  // 單機內部依序執行，保護單機飛控狀態機與任務切換免於競爭，省去內部鎖開銷。
  callback_group_MutuallyExclusive_ = rclcpp::Node::create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  std::string action_name = "/px4_" + std::to_string(UAV_ID_) + "/navigate";
  Action_servers_ = rclcpp_action::create_server<FlightAction>(
      this, action_name,
      // (1) handle_goal: 接收 uuid 和 goal_msg
      [this](const rclcpp_action::GoalUUID &uuid,
             std::shared_ptr<const FlightAction::Goal> goal)
          -> rclcpp_action::GoalResponse {
        return this->HandleGoal(uuid, goal);
      },

      // (2) handle_cancel: 接收 goal_handle
      [this](
          const std::shared_ptr<rclcpp_action::ServerGoalHandle<FlightAction>>
              goal_handle) -> rclcpp_action::CancelResponse {
        return this->HandleCancel(goal_handle);
      },

      // (3) handle_accepted: 接收 goal_handle
      [this](
          const std::shared_ptr<rclcpp_action::ServerGoalHandle<FlightAction>>
              goal_handle) -> void { this->HandleAccepted(goal_handle); },
      rcl_action_server_get_default_options(),
      callback_group_MutuallyExclusive_);

  // 建立傳送 ready 給 main control 的 publisher
  rclcpp::QoS ready_qos(1);
  ready_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  ready_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
  rclcpp::PublisherOptions ready_options;
  ready_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
  ready_pub_ = this->create_publisher<std_msgs::msg::UInt16>(
      "/swarm/uav_ready", ready_qos, ready_options);

  //  建立 PX4 指令Publisher
  rclcpp::PublisherOptions PX4_pub_options;
  PX4_pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;

  std::string Publish_name =
      "/px4_" + std::to_string(UAV_ID_) + "/fmu/in/vehicle_command";
  _vehicle_command_pub = this->create_publisher<px4_msgs::msg::VehicleCommand>(
      Publish_name, 10, PX4_pub_options);

  // 2. 監控 pre_flight_checks_pass，搭配 doRegister() 結果決定是否廣播就緒
  rclcpp::SubscriptionOptions PX4_sub_options;
  PX4_sub_options.callback_group = callback_group_MutuallyExclusive_;
  PX4_sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;

  std::string Sub_name =
      "/px4_" + std::to_string(UAV_ID_) + "/fmu/out/vehicle_status_v4";
  status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
      Sub_name, rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        if (!status_sub_) return;

        if (mode_registered_.load(std::memory_order_acquire) &&
            msg->pre_flight_checks_pass) {
          std_msgs::msg::UInt16 ready_msg;
          ready_msg.data = static_cast<uint16_t>(UAV_ID_);
          ready_pub_->publish(ready_msg);

          RCLCPP_INFO(
              this->get_logger(),
              "[UAV %d] 起飛前檢查通過，已廣播就緒訊號至 /swarm/uav_ready",
              UAV_ID_);

          status_sub_.reset();  // 關閉Subscription
        }
      },
      PX4_sub_options);
}

void UAVNode::initPX4Connection() {
  // 1. 分段等待 FMU（總上限 60 秒；拆成每次 5 秒可讓 Ctrl+C 迅速中止）
  constexpr int kMaxWaitAttempts = 12;
  bool fmu_ready = false;
  for (size_t attempt = 1; attempt <= kMaxWaitAttempts && rclcpp::ok();
       ++attempt) {
    if (px4_ros2::waitForFMU(*this, std::chrono::seconds(5),
                             std::chrono::seconds(5), "")) {
      fmu_ready = true;
      break;
    }
    RCLCPP_WARN(this->get_logger(),
                "[UAV %d] 等待 PX4 FMU 中 嘗試次數(%d/%d)...", UAV_ID_, attempt,
                kMaxWaitAttempts);
  }

  if (!fmu_ready) {
    RCLCPP_FATAL(this->get_logger(), "[UAV %d] 等待 PX4 FMU 連線超時！",
                 UAV_ID_);
    return;
  }

  // 2. 向 PX4 註冊 Executor（多機高負載下加上重試）
  constexpr int kMaxRegAttempts = 5;
  bool registered = false;
  for (size_t attempt = 1; attempt <= kMaxRegAttempts && rclcpp::ok();
       ++attempt) {
    if (_goto_executor->doRegister()) {
      registered = true;
      break;
    }
    RCLCPP_WARN(
        this->get_logger(),
        "[UAV %d] 註冊 MyGotoExecutor 失敗，即將重試，嘗試次數 (%d/%d)...",
        UAV_ID_, attempt, kMaxRegAttempts);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  if (!registered) {
    RCLCPP_FATAL(this->get_logger(), "[UAV %d] 註冊 MyGotoExecutor 最終失敗！",
                 UAV_ID_);
    return;
  }

  // 3. 取得 mode_id 後才立起旗標，讓 status_sub_ 在 pre_flight_checks_pass
  // 時發送 ready
  mode_id = _goto_executor->ownedMode().id();
  mode_registered_.store(true, std::memory_order_release);

  RCLCPP_INFO(
      this->get_logger(),
      "[UAV %d] MyGotoExecutor 註冊成功 (mode_id=%d)，等待起飛前檢查通過...",
      UAV_ID_, mode_id);
}

rclcpp_action::GoalResponse UAVNode::HandleGoal(
    const rclcpp_action::GoalUUID &uuid,
    FlightAction::Goal::ConstSharedPtr goal) {
  RCLCPP_INFO(this->get_logger(), "UAV %d Received goal request", UAV_ID_);
  (void)uuid;

  // 檢查軌跡是否為空
  if (goal->trajectory.points.empty()) {
    RCLCPP_ERROR(this->get_logger(), "[UAV %d] 軌跡為空，拒絕任務", UAV_ID_);
    return rclcpp_action::GoalResponse::REJECT;
  }

  Trajectorys_UAV_flight_plan_ =
      std::shared_ptr<const nav_msgs::msg::Trajectory>(goal,
                                                       &(goal->trajectory));
  start_time_ = goal->start_time;

  Waypoint_num_ = Trajectorys_UAV_flight_plan_->points.size();
  UAV_result_status_->trajectoryresult.points.reserve(Waypoint_num_);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse UAVNode::HandleCancel(
    const std::shared_ptr<FlightGoalHandle> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "[UAV %d] 收到取消任務請求", UAV_ID_);
  (void)goal_handle;

  return rclcpp_action::CancelResponse::ACCEPT;
}

void UAVNode::HandleAccepted(
    const std::shared_ptr<FlightGoalHandle> goal_handle) {
  // 更新當前控制的 handle，用於 cancel 判斷
  Current_goal_handle_ = goal_handle;
  // 開一條獨立的 Thread 來啟動 ExecuteFlight
  // 這條 Thread 存在的時間極短，只要 ExecuteFlight 執行完 (幾微秒)
  // 它就會自動消滅，使其盡快返回
  std::thread{[this, goal_handle]() { ExecuteFlight(goal_handle); }}.detach();
}

void UAVNode::ExecuteFlight(
    const std::shared_ptr<FlightGoalHandle> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "[UAV %d] 開始執行飛行任務，總航點數: %d",
              UAV_ID_, Waypoint_num_);

  // 1. 取得內部 points 陣列的參考 (ROS 2 ENU 座標系)
  const std::vector<nav_msgs::msg::TrajectoryPoint> &Trajectory_points_plan =
      Trajectorys_UAV_flight_plan_->points;

  // 清空上一回合的 Result 誤差紀錄並預留空間
  UAV_result_status_->waypoint_time_difference.clear();
  UAV_result_status_->waypoint_distance_difference.clear();
  UAV_result_status_->waypoint_time_difference.reserve(Waypoint_num_);
  UAV_result_status_->waypoint_distance_difference.reserve(Waypoint_num_);

  // 確保模擬器結果陣列大小對齊航點數量
  UAV_result_status_->trajectoryresult.points.resize(Waypoint_num_);

  // =========================================================================
  // 2. 座標與時間轉換：nav_msgs::msg::TrajectoryPoint (ENU) -> WaypointTask
  // (NED)
  // =========================================================================
  std::vector<WaypointTask> px4_tasks;
  px4_tasks.reserve(Waypoint_num_);

  double prev_stamp_s = 0.0;

  for (int i = 0; i < Waypoint_num_; ++i) {
    const nav_msgs::msg::TrajectoryPoint &pt_enu = Trajectory_points_plan[i];
    WaypointTask task;

    // (a) 位置轉換：ROS 2 ENU (x:東, y:北, z:上) -> PX4 NED (x:北, y:東, z:下)
    task.target_offset_ned_m = Eigen::Vector3f(
        static_cast<float>(pt_enu.pose.position.y),  // North = ENU_Y
        static_cast<float>(pt_enu.pose.position.x),  // East  = ENU_X
        static_cast<float>(-pt_enu.pose.position.z)  // Down  = -ENU_Z
    );

    // (b) 期望時間提取：由 header.stamp 轉換為秒數
    // 若 header.stamp 存的是「該段航程期望花費時間 (Duration)」，直接使用
    // stamp_s； 若存的是「從 0 開始累加的相對時間戳」，可改用 (stamp_s -
    // prev_stamp_s)
    const double stamp_s =
        static_cast<double>(pt_enu.header.stamp.sec) +
        static_cast<double>(pt_enu.header.stamp.nanosec) * 1e-9;

    // 此處以 header.stamp 直接代表該點期望花費時間為例 (並設 0.5
    // 秒最小防呆下限)
    task.desired_duration_s = static_cast<float>(std::max(stamp_s, 0.5));
    // 此處以 header.stamp 代表從 0 開始累加的相對時間戳」
    task.desired_duration_s =
        static_cast<float>(std::max(stamp_s - prev_stamp_s, 0.5));
    prev_stamp_s = stamp_s;

    // (c) 預設讓機頭朝向飛行方向
    task.face_direction_of_travel = true;

    px4_tasks.push_back(task);
  }

  // 3. 將轉換好的 NED 航點任務陣列注入給 PX4 Executor
  _goto_executor->setWaypoints(px4_tasks);

  // =========================================================================
  // 4. 註冊 CompletedCallback：當 PX4 飛完所有航點或異常中斷時自動觸發
  // =========================================================================
  _goto_executor->setOnCompletedCallback(
      [this, goal_handle](px4_ros2::Result px4_result,
                          const std::vector<WaypointTelemetryReport> &reports) {
        // (a) 停止 Feedback Timer
        if (_feedback_timer) {
          _feedback_timer->cancel();
        }

        // (b) 將每個航點抵達時的 PX4 遙測報告寫回 Trajectorys_UAV_result_ 與
        // UAV_result_status_
        const size_t report_count =
            std::min(reports.size(), static_cast<size_t>(Waypoint_num_));
        for (size_t i = 0; i < report_count; ++i) {
          const auto &rep = reports[i];

          // 將抵達瞬間的實際位置與速度由 PX4 (NED) 轉回 ROS 2 (ENU)
          // 存入結果陣列
          auto &res_pt = UAV_result_status_->trajectoryresult.points[i];
          res_pt.pose.position.x =
              rep.actual_position_ned_m.y();  // East  = NED_Y
          res_pt.pose.position.y =
              rep.actual_position_ned_m.x();  // North = NED_X
          res_pt.pose.position.z =
              -rep.actual_position_ned_m.z();  // Up    = -NED_Z

          res_pt.velocity.linear.x = rep.actual_velocity_ned_m_s.y();
          res_pt.velocity.linear.y = rep.actual_velocity_ned_m_s.x();
          res_pt.velocity.linear.z = -rep.actual_velocity_ned_m_s.z();

          // 寫入實際抵達花費時間至 header.stamp
          const int32_t sec = static_cast<int32_t>(rep.actual_duration_s);
          const uint32_t nanosec =
              static_cast<uint32_t>((rep.actual_duration_s - sec) * 1e9);
          res_pt.header.stamp.sec = sec;
          res_pt.header.stamp.nanosec = nanosec;

          // 記錄此航點的時間誤差 (秒) 與歐氏距離誤差 (公尺)
          UAV_result_status_->waypoint_time_difference.push_back(
              static_cast<float>(rep.time_error_s));
          UAV_result_status_->waypoint_distance_difference.push_back(
              rep.position_error_norm_m);
        }

        // 計算平均絕對時間和距離誤差
        const auto &time_diffs = UAV_result_status_->waypoint_time_difference;
        const double sum = std::transform_reduce(
            time_diffs.begin(), time_diffs.end(), 0.0, std::plus<>(),
            [](double val) { return std::fabs(val); });
        UAV_result_status_->waypoint_time_difference_all =
            time_diffs.empty() ? 0.0
                               : (sum / static_cast<double>(time_diffs.size()));

        const auto &dist_diffs =
            UAV_result_status_->waypoint_distance_difference;
        const double sum2 = std::transform_reduce(
            dist_diffs.begin(), dist_diffs.end(), 0.0, std::plus<>(),
            [](double val) { return std::fabs(val); });
        UAV_result_status_->waypoint_distance_difference_all =
            dist_diffs.empty()
                ? 0.0
                : (sum2 / static_cast<double>(dist_diffs.size()));

        // (c) 檢查是否因 Client 取消而結束
        if (goal_handle->is_canceling()) {
          RCLCPP_WARN(this->get_logger(),
                      "[UAV %d] 飛行任務已取消，回傳目前累積結果", UAV_ID_);
          goal_handle->canceled(UAV_result_status_);
          return;
        }

        // (d) 判斷最終目標達成或失敗 (碰撞 / 斷線 / 拒絕切換)
        if (px4_result == px4_ros2::Result::Success) {
          RCLCPP_INFO(this->get_logger(),
                      "[UAV %d] 抵達最後目標！任務成功，寫回 Result", UAV_ID_);
          goal_handle->succeed(UAV_result_status_);
        } else {
          RCLCPP_ERROR(this->get_logger(),
                       "[UAV %d] 飛行異常中斷 (%s)，回傳 Abort", UAV_ID_,
                       px4_ros2::resultToString(px4_result));
          goal_handle->abort(UAV_result_status_);
        }
      });

  // =========================================================================
  // 5. 啟動 非同步 Timer：定期檢查 Cancel 並發布即時遙測 Feedback
  // =========================================================================
  _feedback_timer = this->create_timer(
      std::chrono::milliseconds(1000),  // 1Hz (1000ms)
      [this, goal_handle]() {
        // (a) 檢查 Client 是否發送了取消請求
        if (goal_handle->is_canceling()) {
          if (_feedback_timer) {
            _feedback_timer->cancel();
          }
          _goto_executor->onDeactivate(
              px4_ros2::ModeExecutorBase::DeactivateReason::Other);
          goal_handle->canceled(UAV_result_status_);
          return;
        }

        // (b) 從 PX4 Executor 讀取即時模擬器位置與速度 (NED)
        const Eigen::Vector3f curr_pos_ned =
            _goto_executor->getCurrentPositionNed();
        const Eigen::Vector3f curr_vel_ned =
            _goto_executor->getCurrentVelocityNed();

        // (c) 轉換為 ROS 2 ENU 座標並填入 UAV_feedback_->trajectorypoint
        auto &fb_pt = UAV_feedback_->trajectorypoint;
        fb_pt.header.stamp = this->get_clock()->now();
        fb_pt.header.frame_id = "map";

        fb_pt.pose.position.x = curr_pos_ned.y();   // East  = NED_Y
        fb_pt.pose.position.y = curr_pos_ned.x();   // North = NED_X
        fb_pt.pose.position.z = -curr_pos_ned.z();  // Up    = -NED_Z

        fb_pt.velocity.linear.x = curr_vel_ned.y();
        fb_pt.velocity.linear.y = curr_vel_ned.x();
        fb_pt.velocity.linear.z = -curr_vel_ned.z();

        // (d) 發布 Feedback 給 Action Client
        goal_handle->publish_feedback(UAV_feedback_);
      });

  // =========================================================================
  // 6. 啟動 PX4 Executor 狀態機 (非阻塞，立即返回並釋放 Thread)
  // =========================================================================
  std::shared_ptr<rclcpp::Time> last_cmd_time =
      std::make_shared<rclcpp::Time>(0, 0, this->get_clock()->get_clock_type());
  _start_timer = this->create_timer(
      std::chrono::milliseconds(50), [this, last_cmd_time]() {
        // 檢查 Executor 實例是否已取得控制權（進入 onActivate）
        if (_goto_executor->isInCharge()) {
          _start_timer->cancel();
          return;
        }
        const rclcpp::Time now = this->now();
        if (now < start_time_) {
          return;  // 尚未到共同起飛時間戳
        }
        // 到點後每 300ms 重發一次切模式指令，直到 isInCharge() 變 true
        if (last_cmd_time->nanoseconds() == 0 ||
            (now - *last_cmd_time).seconds() >= 0.3) {
          *last_cmd_time = now;
          PX4StartFlight();
        }
      });
}

void UAVNode::PX4StartFlight() {
  px4_msgs::msg::VehicleCommand cmd{};
  cmd.timestamp = 0;  // 使用Gazebo 模擬時間
  cmd.command = px4_msgs::msg::VehicleCommand::
      VEHICLE_CMD_SET_NAV_STATE;  // 用來直接切換 PX4 的內部狀態機
  cmd.param1 = static_cast<float>(mode_id);
  cmd.param2 = 0.0f;
  cmd.param3 = 0.0f;

  // 因為 topic 已隔離在 /px4_1/... 下，target_system 設 0 (廣播) 可避免多機 ID
  // 不符被忽略
  cmd.target_system = 0;     // Broadcast 模式，已用namespace將通訊通道分開
  cmd.target_component = 1;  // 指定目標組件 ID 1代表飛控主程式本身。
  cmd.source_system = 1;  // 標示發送端（機載電腦或地面站）的系統與組件 ID，供
                          // PX4 回傳執行結果（VehicleCommandAck）時辨識來源。
  cmd.source_component = 1;
  cmd.from_external = true;  // 指令來自飛控外部（ROS 2 node）

  _vehicle_command_pub->publish(cmd);
  RCLCPP_INFO(this->get_logger(),
              "已發送切模式指令，等待 PX4 進入 onActivate()...");
}

RCLCPP_COMPONENTS_REGISTER_NODE(UAVNode)