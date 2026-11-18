#include "physical_simulation/PX4_executor.hpp"

MyGotoExecutor::MyGotoExecutor(MyGotoMode &owned_mode, bool enable_takeoff_and_rtl)
    : ModeExecutorBase(
          px4_ros2::ModeExecutorBase::Settings{}.activate(
              px4_ros2::ModeExecutorBase::Settings::Activation::ActivateAlways),
          owned_mode),
      _node(owned_mode.node()),
      _custom_mode(owned_mode),
      _enable_takeoff_and_rtl(enable_takeoff_and_rtl)
{
}

void MyGotoExecutor::onActivate()
{
    if (_enable_takeoff_and_rtl)
    {
        // 進入解鎖狀態
        runState(State::Arming, px4_ros2::Result::Success);
    }
    else
    {
        // 若無人機已在空中，略過起飛直接進入自訂 Goto 模式
        runState(State::MyMode, px4_ros2::Result::Success);
    }
}

void MyGotoExecutor::onDeactivate(DeactivateReason reason) {}

// 狀態機執行引擎
void MyGotoExecutor::runState(State state, px4_ros2::Result previous_result)
{
    // 檢查上一階段是否順利完成
    if (previous_result != px4_ros2::Result::Success)
    {
        RCLCPP_ERROR(_node.get_logger(), "狀態機異常中斷，前一狀態回傳失敗: %s", resultToString(previous_result));
        if (_on_completed_callback)
        {
            _on_completed_callback(previous_result, _custom_mode.getTelemetryReports());
        }
        return;
    }

    RCLCPP_DEBUG(_node.get_logger(), "準備執行狀態機階段: %i", static_cast<int>(state));

    switch (state)
    {
    case State::Reset:
        break;
    case State::Arming:
        RCLCPP_INFO(_node.get_logger(), "正在發送解鎖 (Arm) 指令...");
        // 呼叫 ModeExecutorBase 內建的 arm()，完成後自動跳到 State::TakingOff
        arm([this](px4_ros2::Result result)
            { runState(State::TakingOff, result); });
        break;

    case State::TakingOff:
        // 排程 PX4 起飛。抵達起飛高度後觸發 lambda Callback，轉向 MyMode
        RCLCPP_INFO(_node.get_logger(), "正在發送解鎖 (TakingOff) 指令...");
        takeoff([this](px4_ros2::Result result)
                { 
                    RCLCPP_INFO(_node.get_logger(), "TakingOff 結果: %s",px4_ros2::resultToString(result));
                    runState(State::MyMode, result); });
        break;

    case State::MyMode:
        // 排程啟動自訂的 MyGotoMode
        // MyGotoMode 內部會遍歷所有 Waypoints，全部完成後才觸發此處的回呼函式
        RCLCPP_INFO(_node.get_logger(), "開始自訂義飛行");
        scheduleMode(ownedMode().id(),
                     [this](px4_ros2::Result result)
                     {
                         RCLCPP_INFO(_node.get_logger(), "所有路徑點完成 (%s)", resultToString(result));
                         if (_enable_takeoff_and_rtl)
                         {
                             RCLCPP_INFO(_node.get_logger(), "開始返航");
                             runState(State::RTL, result); // 飛完去返航
                         }
                         else
                         {
                             // 若不啟用返航，至此任務完全結束，觸發 Composable Node 回呼
                             if (_on_completed_callback)
                                 _on_completed_callback(result, _custom_mode.getTelemetryReports());
                         }
                     });
        break;

    case State::RTL:
        // 排程自動返航降落
        rtl([this](px4_ros2::Result result)
            { runState(State::WaitUntilDisarmed, result); });
        break;

    case State::WaitUntilDisarmed:
        // 等待著陸後螺旋槳上鎖 (Disarm)
        waitUntilDisarmed([this](px4_ros2::Result result)
                          {
                              RCLCPP_INFO(_node.get_logger(), "返航成功，完整飛行任務結束 (%s)", resultToString(result));
                              if (_on_completed_callback)
                                  _on_completed_callback(result, _custom_mode.getTelemetryReports()); });
        break;
    }
}

// ================= 外部 Composable Node 呼叫介面轉發 =================
void MyGotoExecutor::setWaypoints(const std::vector<WaypointTask> &waypoints)
{
    _custom_mode.setWaypoints(waypoints);
}
void MyGotoExecutor::setOnCompletedCallback(CompletedCallback callback)
{
    _on_completed_callback = std::move(callback);
}
// const std::vector<WaypointTelemetryReport> &MyGotoExecutor::getTelemetryReports() const
// {
//     return _custom_mode.getTelemetryReports();
// }
Eigen::Vector3f MyGotoExecutor::getCurrentPositionNed() const
{
    return _custom_mode.getCurrentPositionNed();
}
Eigen::Vector3f MyGotoExecutor::getCurrentVelocityNed() const
{
    return _custom_mode.getCurrentVelocityNed();
}