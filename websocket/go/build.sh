#!/bin/bash
set -e

cd "$(dirname "$0")"

ARCH=$(uname -m)

go mod tidy

mkdir -p ../cpp/lib/$ARCH ../cpp/include ../ros2_ws/src/websocket_client_ros/lib/$ARCH ../ros2_ws/src/websocket_client_ros/include
go build -buildmode=c-shared -o ../cpp/lib/$ARCH/libwebsocket_client.so websocket_client.go
mv ../cpp/lib/$ARCH/libwebsocket_client.h ../cpp/include/
cp ../cpp/include/libwebsocket_client.h ../ros2_ws/src/websocket_client_ros/include/
cp ../cpp/lib/$ARCH/libwebsocket_client.so ../ros2_ws/src/websocket_client_ros/lib/$ARCH/

echo "Go库编译完成 (架构: $ARCH)"
