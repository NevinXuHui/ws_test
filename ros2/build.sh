#!/bin/bash
set -e

cd "$(dirname "$0")"

# 先构建Go库
echo "=== 构建Go库 ==="
cd ..
./build.sh
cd ros2

# 构建ROS2包
echo ""
echo "=== 构建ROS2包 ==="
source /opt/ros/humble/setup.bash
colcon build

echo ""
echo "=== 构建完成 ==="
echo "运行: ./run.sh"
