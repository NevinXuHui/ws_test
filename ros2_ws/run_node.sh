#!/bin/bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/../cpp
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"
ros2 run websocket_client_ros websocket_node
