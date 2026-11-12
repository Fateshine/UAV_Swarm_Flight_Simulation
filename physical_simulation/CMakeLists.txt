cmake_minimum_required(VERSION 3.8)
project(physical_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_C_STANDARD 99)              
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 17) 
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# 尋找自定義 Action Package
find_package(custom_interfaces REQUIRED)

# 尋找 PX4 ROS 2 C++ 介面庫與 Eigen 矩陣運算庫
find_package(px4_ros2_cpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# ==========================================
# 元件 1 Main Control (Action Client) SHARED 動態函式庫
# ==========================================
add_library(Main_control_component SHARED src/Main_control.cpp)

# 設定標頭檔路徑 (讓 cpp 找得到 hpp)
# Component 必須使用 PUBLIC 與 Generator Expressions 以供容器正確載入與連結
target_include_directories(Main_control_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 連結所有需要的依賴包
ament_target_dependencies(Main_control_component
  rclcpp
  rclcpp_action
  rclcpp_components
  nav_msgs
  geometry_msgs
  custom_interfaces
)

# 註冊外掛節點
# PLUGIN: C++ Class 完整名稱 
# EXECUTABLE: ROS 2 會自動包裝一個獨立執行檔，使其依舊能用 ros2 run 測試
rclcpp_components_register_node(Main_control_component
  PLUGIN "MainControl" 
  EXECUTABLE Main_control_node
)

# ==========================================
# 元件 2：UAV Node (Action Server Component) SHARED 動態函式庫
# 將 PX4 的 Mode 與 Executor 實作檔一起編譯進 UAV_node_component
# ==========================================
add_library(UAV_node_component SHARED 
  src/UAV_node.cpp
  src/PX4_goto_mode.cpp
  src/PX4_executor.cpp
)

# 設定標頭檔路徑 (讓 cpp 找得到 hpp)
# Component 必須使用 PUBLIC 與 Generator Expressions 以供容器正確載入與連結
target_include_directories(UAV_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# 連結所有需要的依賴包
ament_target_dependencies(UAV_node_component
  rclcpp
  rclcpp_action
  rclcpp_components
  nav_msgs
  geometry_msgs
  custom_interfaces
  px4_ros2_cpp
  Eigen3
)

# 註冊外掛節點
# PLUGIN: C++ Class 完整名稱 
rclcpp_components_register_node(UAV_node_component
  PLUGIN "UAVNode"
  EXECUTABLE UAV_node
)

# ==========================================
# 安裝與輸出設定
# ==========================================
# Component 必須安裝在 library 區塊中，讓 rclcpp_components 能透過檔案系統動態載入
install(TARGETS
  Main_control_component
  UAV_node_component
  ARCHIVE DESTINATION lib # 靜態函式庫位置
  LIBRARY DESTINATION lib # 動態/共用函式庫位置
  RUNTIME DESTINATION bin # 可執行檔位置
)

# 安裝 launch 資料夾到 share 目錄下
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
