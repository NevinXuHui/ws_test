#!/bin/bash
set -e

echo "=== 构建WebSocket客户端库 ==="

# 安装依赖
go mod tidy

# 编译共享库
echo "编译Go共享库..."
go build -buildmode=c-shared -o libwebsocket_client.so websocket_client.go

# 编译C++示例
if command -v g++ &> /dev/null; then
    echo "编译C++示例..."
    g++ -std=c++11 -o example example.cpp -L. -lwebsocket_client -lpthread
fi

echo ""
echo "=== 构建完成 ==="
echo "运行: LD_LIBRARY_PATH=. ./example"
