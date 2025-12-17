#!/bin/bash

cd "$(dirname "$0")"

# 自动检测ROS2版本
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/iron/setup.bash ]; then
    source /opt/ros/iron/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/galactic/setup.bash ]; then
    source /opt/ros/galactic/setup.bash
else
    echo "未找到ROS2安装"
    exit 1
fi

colcon build --packages-up-to websocket_client_ros

echo "ROS2构建完成！"
