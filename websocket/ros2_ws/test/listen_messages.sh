#!/bin/bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"
export RCUTILS_COLORIZED_OUTPUT=1

echo "监听WebSocket消息..."
ros2 topic echo /homi_speech/sigc_event_topic homi_speech_interface/msg/SIGCEvent --field event
