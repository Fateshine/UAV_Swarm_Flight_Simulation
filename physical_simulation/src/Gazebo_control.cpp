#include "physical_simulation/Gazebo_control.hpp"

#include <chrono>
#include <utility>

using namespace std::chrono_literals;

GazeboSimController::GazeboSimController(
    rclcpp::Node& node, rclcpp::CallbackGroup::SharedPtr callback_group)
    : logger_(node.get_logger()), callback_group_(std::move(callback_group)) {
  // 使用 ROS 2 Jazzy 標準的 ServicesQoS（Reliable + Volatile，確保 Service
  // 請求不掉包）
  const rclcpp::ServicesQoS srv_qos = rclcpp::ServicesQoS();

  // 將 3 個 Service Client 掛載到父節點與指定的 CallbackGroup 上
  reset_client_ = node.create_client<ResetSimulation>(
      "/gzserver/reset_simulation", srv_qos, callback_group_);

  get_state_client_ = node.create_client<GetSimulationState>(
      "/gzserver/get_simulation_state", srv_qos, callback_group_);

  set_state_client_ = node.create_client<SetSimulationState>(
      "/gzserver/set_simulation_state", srv_qos, callback_group_);
}

void GazeboSimController::resetSimulation(uint8_t scope,
                                          ResultCallback on_done) {
  // 1. 快速檢查服務是否在線（最多等 500ms）
  //    採短逾時快速失敗（Fail-fast），避免 GzServer 尚未啟動或崩潰時卡住
  //    MainControl 主流程
  if (!reset_client_->wait_for_service(500ms)) {
    const std::string err = "Service /gzserver/reset_simulation not available.";
    RCLCPP_ERROR(logger_, "%s", err.c_str());
    if (on_done) {
      on_done(false);
    }
    return;
  }

  // 2. 組裝請求訊息，設定重置範圍
  std::shared_ptr<simulation_interfaces::srv::ResetSimulation_Request> request =
      std::make_shared<ResetSimulation::Request>();
  request->scope = scope;

  // 3. 以非同步方式送出請求
  //    【重要】絕不在這裡呼叫 future.get() 同步死等！
  //    因為若此函式本身是在 ROS 2 Callback 內被呼叫，同步等待會霸佔 Executor
  //    執行緒， 導致回應永遠無法被處理而發生死鎖；改由 ROS 2
  //    收到回應後自動觸發下方的 Lambda。
  reset_client_->async_send_request(
      request,
      [this, on_done](rclcpp::Client<ResetSimulation>::SharedFuture future) {
        // 此 Lambda 執行時代表回應已經抵達，此時呼叫 future.get()
        // 會立即取得結果不阻塞
        const std::shared_ptr<
            simulation_interfaces::srv::ResetSimulation_Response>
            response = future.get();

        // simulation_interfaces 統一透過回傳欄位內的 result.result 判斷是否成功
        const bool ok = (response->result.result == SimResult::RESULT_OK);
        if (ok) {
          RCLCPP_INFO(logger_, "ResetSimulation succeeded.");
        } else {
          RCLCPP_ERROR(logger_, "ResetSimulation failed: %s",
                       response->result.error_message.c_str());
        }

        // 將結果通知呼叫端（例如 MainControl 收到後才進行下一階段任務初始化）
        if (on_done) {
          on_done(ok);
        }
      });
}

void GazeboSimController::getSimulationState(StateCallback on_done) {
  // 1. 檢查服務是否可用
  if (!get_state_client_->wait_for_service(500ms)) {
    const std::string err =
        "Service /gzserver/get_simulation_state not available.";
    RCLCPP_ERROR(logger_, "%s", err.c_str());
    if (on_done) {
      on_done(false, SimulationState{});
    }
    return;
  }

  // 2. GetSimulationState 的 Request 為空結構，仍需建立空的 shared_ptr 傳入
  std::shared_ptr<simulation_interfaces::srv::GetSimulationState_Request>
      request = std::make_shared<GetSimulationState::Request>();

  // 3. 非同步送出查詢請求並處理回傳狀態
  get_state_client_->async_send_request(
      request,
      [this, on_done](rclcpp::Client<GetSimulationState>::SharedFuture future) {
        const std::shared_ptr<
            simulation_interfaces::srv::GetSimulationState_Response>
            response = future.get();
        const bool ok = (response->result.result == SimResult::RESULT_OK);
        if (ok) {
          RCLCPP_INFO(logger_, "Current simulation state: %s (%u)",
                      stateToString(response->state.state).c_str(),
                      response->state.state);
        } else {
          RCLCPP_ERROR(logger_, "GetSimulationState failed: %s",
                       response->result.error_message.c_str());
        }

        // 回傳查詢結果與目前狀態碼給呼叫端
        if (on_done) {
          on_done(ok, response->state);
        }
      });
}

void GazeboSimController::setSimulationState(uint8_t target_state,
                                             ResultCallback on_done) {
  // 1. 檢查服務是否可用
  if (!set_state_client_->wait_for_service(500ms)) {
    const std::string err =
        "Service /gzserver/set_simulation_state not available.";
    RCLCPP_ERROR(logger_, "%s", err.c_str());
    if (on_done) {
      on_done(false);
    }
    return;
  }

  // 2. 填入目標模擬狀態（注意 SimulationState 是一個內嵌訊息，狀態值放在
  // state.state）
  std::shared_ptr<simulation_interfaces::srv::SetSimulationState_Request>
      request = std::make_shared<SetSimulationState::Request>();
  request->state.state = target_state;

  // 3. 非同步送出狀態切換請求
  set_state_client_->async_send_request(
      request, [this, target_state, on_done](
                   rclcpp::Client<SetSimulationState>::SharedFuture future) {
        const std::shared_ptr<
            simulation_interfaces::srv::SetSimulationState_Response>
            response = future.get();
        const bool ok = (response->result.result == SimResult::RESULT_OK);

        if (ok) {
          RCLCPP_INFO(logger_, "SetSimulationState -> %s succeeded.",
                      stateToString(target_state).c_str());
        } else {
          RCLCPP_ERROR(logger_, "SetSimulationState failed: %s",
                       response->result.error_message.c_str());
        }

        if (on_done) {
          on_done(ok);
        }
      });
}

std::string GazeboSimController::stateToString(uint8_t state) {
  switch (state) {
    case SimulationState::STATE_STOPPED:
      return "STOPPED";
    case SimulationState::STATE_PLAYING:
      return "PLAYING";
    case SimulationState::STATE_PAUSED:
      return "PAUSED";
    case SimulationState::STATE_QUITTING:
      return "QUITTING";
    default:
      return "UNKNOWN";
  }
}