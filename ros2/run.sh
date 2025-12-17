#!/bin/bash

cd "$(dirname "$0")"

# 检查是否已构建
if [ ! -d "install" ]; then
    echo "未找到构建文件，先执行构建..."
    ./build.sh
fi

# 设置环境
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/../cpp

CONFIG_PATH=$(realpath ../cpp/config.json)

echo "=== 运行WebSocket ROS2节点 ==="
ros2 run websocket_client_ros websocket_node --ros-args -p config_path:=$CONFIG_PATH
