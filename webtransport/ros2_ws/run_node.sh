#!/bin/bash
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch webtransport_client_ros webtransport.launch.py
