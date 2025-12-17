#!/bin/bash
source /opt/ros/humble/setup.bash

# 构建WebSocket C库
echo "构建 WebSocket C库..."
cd ../
./build.sh
cd ros2_ws

# 构建接口包
echo "构建 homi_speech_interface..."
colcon build --packages-select homi_speech_interface

# 设置环境
source install/setup.bash

# 构建WebSocket节点
echo "构建 websocket_client_ros..."
colcon build --packages-select websocket_client_ros

echo "构建完成！"
