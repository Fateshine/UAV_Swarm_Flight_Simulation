import math
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ros_gz_sim.actions import GzServer


def generate_launch_description():
    # =========================================================================
    # 1. 取得套件路徑與設定檔路徑
    # =========================================================================
    pkg_name = "physical_simulation"
    pkg_dir = Path(get_package_share_directory(pkg_name))
    # ros_gz_sim_dir = Path(get_package_share_directory("ros_gz_sim"))
    config_path = pkg_dir / "launch" / "Config.yaml"

    # 讀取 YAML 檔中的無人機數量
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    UAV_count = config["UAV"]["UAV_count"]
    use_sim_time = config["System"]["use_sim_time"]

    # PX4-Autopilot 編譯檔與Gazebo模型路徑
    px4_dir = Path.home() / "PX4-Autopilot"
    px4_bin = px4_dir / "build" / "px4_sitl_default" / "bin" / "px4"
    gz_models_dir = px4_dir / "Tools" / "simulation" / "gz" / "models"
    gz_worlds_dir = px4_dir / "Tools" / "simulation" / "gz" / "worlds"
    gz_server_config = px4_dir / "Tools" / "simulation" / "gz" / "server.config"
    world_file = gz_worlds_dir / "default.sdf"

    launch_actions = []

    # =========================================================================
    # 2. 設定 Gazebo 資源環境變數 (讓 ros-gz 找到 PX4 的無人機模型與世界檔)
    # =========================================================================
    set_gz_models = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=f"{gz_models_dir}:{gz_worlds_dir}"
    )
    launch_actions.append(set_gz_models)

    # 指定 PX4 的 Gazebo 設定檔，讓模擬器載入 IMU、氣壓計、GPS 等感測器外掛
    set_gz_config = SetEnvironmentVariable(
        name="GZ_SIM_SERVER_CONFIG_PATH",
        value=str(gz_server_config),
    )
    launch_actions.append(set_gz_config)

    # =========================================================================
    # 3. 啟動 Gazebo Harmonic 使用 GzServer 模式
    # =========================================================================
    gz_server = GzServer(
        # 將 Path 物件轉為字串傳入
        world_sdf_file=str(world_file),
        use_composition=False,
        verbosity_level=3,
    )
    launch_actions.append(gz_server)

    # 啟動 ros_gz_bridge 將 Gazebo 的物理時鐘 (/clock) 同步給所有 ROS 2 節點
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )
    launch_actions.append(clock_bridge)

    # =========================================================================
    # 4. 啟動 Micro-XRCE-DDS Agent (負責 uORB <-> ROS 2 DDS 橋接)
    # =========================================================================
    xrce_agent_process = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        name="micro_xrce_agent",
        output="screen",
    )
    launch_actions.append(xrce_agent_process)

    # =========================================================================
    # 5. 動態啟動多個 PX4 SITL Process 與對應的 ComposableNode
    # =========================================================================
    composable_nodes = []

    # 5.1 建立 MainControl (開啟 use_sim_time 對齊 Gazebo 時間)
    main_control_node = ComposableNode(
        package=pkg_name,
        plugin="MainControl",
        name="main_control",
        parameters=[{"UAV_count": UAV_count}, {"use_sim_time": use_sim_time}],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    composable_nodes.append(main_control_node)

    # 計算方陣排列座標，避免多機生成在同一個點重疊相撞
    grid_cols = math.ceil(math.sqrt(UAV_count))
    spacing_m = 2.0

    for i in range(UAV_count):
        px4_instance_id = i + 1
        spawn_x = (i // grid_cols) * spacing_m
        spawn_y = (i % grid_cols) * spacing_m

        # 5.2 啟動 PX4 SITL 獨立 Process (連線至上面 啟動的 GzServer)
        px4_env = os.environ.copy()
        px4_env["PX4_SYS_AUTOSTART"] = "4001"
        px4_env["PX4_SIM_MODEL"] = "gz_x500"
        px4_env["PX4_GZ_STANDALONE"] = "1"  # 告知 PX4 使用已由 ros_gz_sim 啟動的 Gazebo
        px4_env["PX4_GZ_WORLD"] = (
            "default"  # 必須與 world_file 內的 <world name="..."> 一致
        )
        px4_env["PX4_GZ_MODEL_POSE"] = f"{spawn_x},{spawn_y},0.0,0.0,0.0,0.0"
        px4_env["PX4_UXRCE_DDS_NS"] = f"px4_{px4_instance_id}"

        px4_env["PX4_PARAM_COM_RCL_EXCEPT"] = (
            "4"  # 允許外部/Offboard 模式在無遙控器時解鎖
        )
        px4_env["PX4_PARAM_NAV_DLL_ACT"] = (
            "0"  # 關閉資料鏈路失聯檢查，避免無 QGC 擋住起飛前檢查
        )
        px4_env["PX4_PARAM_CBRK_SUPPLY_CHK"] = (
            "894281"  # 跳過實體供電檢查，解決 system power unavailable
        )
        # 以下暫時使用，之後用真實數據
        px4_env["PX4_PARAM_MPC_XY_VEL_MAX"] = "18.0"
        px4_env["PX4_PARAM_MPC_ACC_HOR"] = "7.0"
        px4_env["PX4_PARAM_MPC_JERK_AUTO"] = "15.0"

        px4_sitl_process = ExecuteProcess(
            cmd=[
                str(px4_bin),
                "-i",
                str(px4_instance_id),
                "-d",
                str(px4_dir / "build" / "px4_sitl_default" / "etc"),
            ],
            cwd=str(px4_dir / "build" / "px4_sitl_default"),
            env=px4_env,
            name=f"px4_sitl_{px4_instance_id}",
            output="screen",
        )
        launch_actions.append(px4_sitl_process)

        # 5.3 建立對應的 UAVNode (開啟 use_sim_time 讓計時器與誤差結算對齊 Gazebo 時間)
        uav_node = ComposableNode(
            package=pkg_name,
            plugin="UAVNode",
            name=f"uav_{px4_instance_id}",
            namespace=f"/px4_{px4_instance_id}",
            parameters=[{"UAV_id": px4_instance_id}, {"use_sim_time": use_sim_time}],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
        composable_nodes.append(uav_node)

    # =========================================================================
    # 6. 建立 Multi-threaded Component Container (延遲 5 秒待 SITL 與 /clock 就緒)
    # =========================================================================
    container = ComposableNodeContainer(
        name="swarm_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=composable_nodes,
        parameters=[{"use_sim_time": use_sim_time}],
        # prefix=["gdb -batch -ex run -ex bt --args"],  # GDB 除錯
        output="screen",
    )

    delayed_container = TimerAction(period=5.0, actions=[container])
    launch_actions.append(delayed_container)

    return LaunchDescription(launch_actions)
