#!/bin/bash

cd "$(dirname "$0")"

ARCH=$(uname -m)

# 自动检测ROS2版本
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    ROS_DISTRO=jazzy
elif [ -f /opt/ros/iron/setup.bash ]; then
    source /opt/ros/iron/setup.bash
    ROS_DISTRO=iron
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    ROS_DISTRO=humble
elif [ -f /opt/ros/galactic/setup.bash ]; then
    source /opt/ros/galactic/setup.bash
    ROS_DISTRO=galactic
elif [ -f /opt/ros/foxy/setup.bash ]; then
    source /opt/ros/foxy/setup.bash
    ROS_DISTRO=foxy
else
    echo "未找到ROS2安装"
    exit 1
fi

INSTALL_DIR=install/${ARCH}/${ROS_DISTRO}

source $INSTALL_DIR/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/src/websocket_client_ros/lib/$ARCH
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"

ros2 launch websocket_client_ros websocket.launch.py
