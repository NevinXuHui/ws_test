#!/bin/bash
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
colcon build --packages-select webtransport_client_ros
