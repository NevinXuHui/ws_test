#!/bin/bash
set -e
cd "$(dirname "$0")"

ARCH=$(uname -m)
echo "=== 编译 WebTransport 全部组件 (${ARCH}) ==="

# 1. 编译 Go C库
echo ">>> 编译 clib..."
cd clib && ./build.sh && cd ..

# 2. 复制库到 ROS2
echo ">>> 复制库到 ROS2..."
ROS_LIB="ros2_ws/src/webtransport_client_ros/lib/${ARCH}"
mkdir -p "$ROS_LIB"
cp "cpp/lib/${ARCH}/libwebtransport_client.so" "$ROS_LIB/"

# 3. 编译 C++ 示例
echo ">>> 编译 C++ 示例..."
cd cpp && make && cd ..

# 4. 编译 ROS2
echo ">>> 编译 ROS2..."
cd ros2_ws && ./build_ros.sh && cd ..

echo "=== 编译完成 ==="
