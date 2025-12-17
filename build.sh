#!/bin/bash
set -e

echo "=== 构建WebSocket客户端库 ==="

cd "$(dirname "$0")"

# 安装Go依赖
cd go
go mod tidy

# 编译共享库到cpp目录
echo "编译Go共享库..."
go build -buildmode=c-shared -o ../cpp/libwebsocket_client.so websocket_client.go
cd ..

# 编译C++示例
if command -v g++ &> /dev/null; then
    echo "编译C++示例..."
    cd cpp
    g++ -std=c++11 -o example example.cpp -L. -lwebsocket_client -lpthread
    cd ..
fi

echo ""
echo "=== 构建完成 ==="
echo "运行: ./run.sh"
