#!/bin/bash

set -e  # 只要編譯或任何步驟失敗就立刻中止，避免誤跑舊的執行檔

# 載入基礎 ROS 2 Jazzy 環境
source /opt/ros/jazzy/setup.bash

WS_DIR=~/ros2_ws
cd "$WS_DIR"

# release 最終優化執行用
BUILD_TYPE="Release" 
# Debug 測試用
# BUILD_TYPE="Debug"

# 若前次曾編譯過其他模式 (如 Debug)，自動清理快取；日常重複執行則維持快速增量編譯
CLEAN_FLAG=""
CACHE_FILE="$WS_DIR/build/physical_simulation/CMakeCache.txt"
if [ -f "$CACHE_FILE" ]; then
    if ! grep -q "CMAKE_BUILD_TYPE:STRING=$BUILD_TYPE" "$CACHE_FILE"; then
        CLEAN_FLAG="--cmake-clean-cache"
    fi
fi

# 自訂義Ros2 interface
colcon build --symlink-install --packages-select custom_interfaces --parallel-workers 4 --cmake-clean-cache --cmake-args -DCMAKE_BUILD_TYPE=Release

# physical_simulation 主系統
colcon build --symlink-install --packages-select physical_simulation --parallel-workers 4 $CLEAN_FLAG --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE

source "$WS_DIR/install/setup.bash"

ros2 launch physical_simulation launch.py