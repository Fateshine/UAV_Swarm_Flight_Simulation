from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # 1. 取得套件路徑與設定檔路徑
    pkg_name = 'physical_simulation'
    # 使用 Path 將字串轉換為路徑物件，並用 / 運算子串接路徑
    pkg_dir = Path(get_package_share_directory(pkg_name))
    config_path = pkg_dir / 'config' / 'swarm_config.yaml'

    # 2. 讀取 YAML 檔中的無人機數量
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    UAV_count = config['swarm']['UAV_count']

    # 3. 建立 Component Container
    composable_nodes = []
    
    main_control_node = ComposableNode(
        package=pkg_name,
        plugin='MainControl', 
        name='main_control',
        parameters=[{'UAV_count': UAV_count}], 
        # 開啟同程序Zero-copy通訊
        extra_arguments=[{'use_intra_process_comms': True}] 
    )
    composable_nodes.append(main_control_node)

    # 4. 根據 yaml 數量，動態生成多個 UAV Node
    for i in range(UAV_count):
        uav_node = ComposableNode(
            package=pkg_name,
            plugin='UAVNode', 
            name=f'uav_{i}', # 自動命名為 uav_0, uav_1, uav_2 ...
            parameters=[{'UAV_id': i}], 
            # 開啟同程序Zero-copy通訊
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        composable_nodes.append(uav_node)

    # 5. 建立 Multi-threaded Component Container，將所有 Node 掛載進去
    container = ComposableNodeContainer(
        name='swarm_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt', # 使用 _mt (多執行緒)
        composable_node_descriptions=composable_nodes,
        output='screen'
    )

    return LaunchDescription([container])