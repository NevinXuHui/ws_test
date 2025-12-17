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
else
    echo "未找到ROS2安装"
    exit 1
fi

BUILD_DIR=build/${ARCH}/${ROS_DISTRO}
INSTALL_DIR=install/${ARCH}/${ROS_DISTRO}
LOG_DIR=log/${ARCH}/${ROS_DISTRO}

echo "编译架构: $ARCH, ROS2版本: $ROS_DISTRO"

colcon build --packages-up-to websocket_client_ros \
    --build-base $BUILD_DIR \
    --install-base $INSTALL_DIR \
    --log-base $LOG_DIR

echo "ROS2构建完成！"
