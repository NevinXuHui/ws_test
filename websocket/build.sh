#!/bin/bash
set -e

echo "=== 构建WebSocket客户端库 ==="

cd "$(dirname "$0")"

# 编译Go共享库
echo "编译Go共享库..."
./go/build.sh

# 编译C++示例
echo "编译C++示例..."
cd cpp && make && cd ..

# 编译ROS2节点
echo "编译ROS2节点..."
./ros2_ws/build_ros.sh

echo ""
echo "=== 构建完成 ==="
echo "运行: ./run.sh"
