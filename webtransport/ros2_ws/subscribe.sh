#!/bin/bash
echo "订阅可靠消息: /webtransport/reliable_recv"
echo "订阅不可靠消息: /webtransport/unreliable_recv"
echo "按 Ctrl+C 退出"
ros2 topic echo /webtransport/reliable_recv &
ros2 topic echo /webtransport/unreliable_recv
