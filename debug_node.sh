#!/bin/bash
cd /mine/Code/ROS/ws_test/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/../cpp
gdb install/websocket_client_ros/lib/websocket_client_ros/websocket_node
