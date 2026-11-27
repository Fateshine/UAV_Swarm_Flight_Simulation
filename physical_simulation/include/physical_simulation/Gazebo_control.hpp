#pragma once
#ifndef GAZEBO_SIM_CONTROLLER_HPP_
#define GAZEBO_SIM_CONTROLLER_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Gazebo Harmonic 透過 ros_gz_sim (GzServer) 原生提供的 ROS 2 標準模擬控制介面
#include "simulation_interfaces/msg/result.hpp"
#include "simulation_interfaces/msg/simulation_state.hpp"
#include "simulation_interfaces/srv/get_simulation_state.hpp"
#include "simulation_interfaces/srv/reset_simulation.hpp"
#include "simulation_interfaces/srv/set_simulation_state.hpp"

/**
 * @class GazeboSimController
 * @brief 用於控制 Gazebo Harmonic 模擬流程（播放 / 暫停 / 停止 / 查詢 /
 * 重置）的輔助類別。
 *
 * 【架構設計說明】
 * 1.
 * 本類別「不繼承」rclcpp::Node，而是以組合（Composition）方式嵌入至父節點（
 * MainControl）中， 共用父節點的生命週期、Executor 與 Logger，避免多開無謂的
 * ROS 2 節點與 DDS 資源消耗。
 * 2. 所有 Service 呼叫皆採用完全「非阻塞（Asynchronous）」設計，搭配 Callback
 * 回傳結果， 確保在 ROS 2 Callback（例如 Timer 或
 * Subscription）中呼叫時絕對不會發生死鎖（Deadlock）。
 */
class GazeboSimController {
 public:
  // 型別別名，簡化呼叫端與實作檔中的冗長命名空間
  using ResetSimulation = simulation_interfaces::srv::ResetSimulation;
  using GetSimulationState = simulation_interfaces::srv::GetSimulationState;
  using SetSimulationState = simulation_interfaces::srv::SetSimulationState;
  using SimulationState = simulation_interfaces::msg::SimulationState;
  using SimResult = simulation_interfaces::msg::Result;

  /**
   * @brief 一般操作完成後的非同步回呼函式簽章（用於 SetSimulationState /
   * ResetSimulation）
   * @param success   true 代表 Gazebo 回傳 RESULT_OK；false
   */
  using ResultCallback = std::function<void(bool success)>;

  /**
   * @brief 狀態查詢完成後的非同步回呼函式簽章（用於 GetSimulationState）
   * @param success   true 代表查詢成功
   * @param state     目前模擬狀態碼（對應 SimulationState::STATE_* 常數）
   */
  using StateCallback =
      std::function<void(bool success, SimulationState state)>;

  /**
   * @brief 建構子：將所有 Service Client 與選用的指令 Topic 掛載到父節點上
   *
   * @param node
   *   父節點參考（在 MainControl 建構子中直接傳入 *this 即可，不需依賴
   * shared_from_this()）。
   *
   * @param callback_group
   *   選用。若父節點運行在 MultiThreadedExecutor 下，建議傳入專屬的
   * MutuallyExclusive CallbackGroup，避免模擬控制的 Service
   * 回應被其他高頻飛控或軌跡回呼阻塞。
   */
  explicit GazeboSimController(rclcpp::Node& node,
                               rclcpp::CallbackGroup::SharedPtr callback_group);

  /**
   * @brief 1. 重置模擬世界（呼叫 /gzserver/reset_simulation）
   *
   * @param scope
   *   重置範圍，預設為
   * ResetSimulation::Request::SCOPE_DEFAULT（依模擬器預設重置）。
   *   其他可用常數可透過指令 `ros2 interface show
   * simulation_interfaces/srv/ResetSimulation` 查閱。
   * @param on_done
   *   選用的完成回呼，若不需後續處理可傳 nullptr。
   *
   * @warning 【搭配 PX4 SITL 重要警告】
   *   切勿在 PX4 飛控進程仍在執行時單獨呼叫此函式重置世界！
   *   因為 Gazebo 會瞬間將機體位置與模擬時間歸零，但 PX4 內部的 EKF2
   * 濾波器與單調時鐘無法歸零， 會導致 EKF2 發散或 Lockstep 鎖死。搭配 PX4
   * 使用時，應由外部腳本先終止並重啟 PX4 進程。
   */
  void resetSimulation(uint8_t scope = ResetSimulation::Request::SCOPE_DEFAULT,
                       ResultCallback on_done = nullptr);

  /**
   * @brief 2. 查詢目前模擬狀態（呼叫 /gzserver/get_simulation_state）
   *
   * @param on_done
   *   選用的完成回呼，會將查到的 state（PLAYING / PAUSED / STOPPED
   * 等）傳回給呼叫端。
   */
  void getSimulationState(StateCallback on_done = nullptr);

  /**
   * @brief 3. 切換模擬狀態（呼叫 /gzserver/set_simulation_state）
   *
   * @param target_state
   *   目標狀態碼，請使用以下標準常數：
   *   - SimulationState::STATE_STOPPED  (0)：停止模擬
   *   - SimulationState::STATE_PLAYING  (1)：開始 / 繼續模擬
   *   - SimulationState::STATE_PAUSED   (2)：暫停模擬（凍結物理引擎與 /clock）
   *   - SimulationState::STATE_QUITTING (3)：要求模擬器結束
   * @param on_done
   *   選用的完成回呼。
   *
   * @note 【搭配 PX4 SITL 提醒】
   *   使用暫停與恢復功能時，請確認 PX4 參數 UXRCE_DDS_SYNCT 已設為
   * 0（關閉牆上時間對時）， 且所有 ROS 2 節點皆啟用
   * use_sim_time，避免恢復播放時觸發 time jump 警告或控制逾時。
   */
  void setSimulationState(uint8_t target_state,
                          ResultCallback on_done = nullptr);

  /**
   * @brief 輔助函式：將 SimulationState 數值狀態碼轉成可讀字串（供 Log
   * 輸出使用）
   */
  static std::string stateToString(uint8_t state);

 private:
  // 借用父節點的 Logger，確保所有 Log 來源名稱與父節點一致（例如
  // [main_control]）
  rclcpp::Logger logger_;

  // 保存 CallbackGroup 參考，供 Client 與 Subscription 綁定使用
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // 3 個對應 Gazebo simulation_interfaces 的 ROS 2 Service Client
  rclcpp::Client<ResetSimulation>::SharedPtr reset_client_;
  rclcpp::Client<GetSimulationState>::SharedPtr get_state_client_;
  rclcpp::Client<SetSimulationState>::SharedPtr set_state_client_;
};

#endif  // GAZEBO_SIM_CONTROLLER_HPP_