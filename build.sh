#!/bin/bash

# WebSocket Client Build Script

set -e

echo "Building WebSocket Client..."

# 检查Go环境
if ! command -v go &> /dev/null; then
    echo "Error: Go is not installed"
    exit 1
fi

# 安装依赖
echo "Installing Go dependencies..."
go mod tidy

# 编译共享库
echo "Building shared library..."
go build -buildmode=c-shared -o libwebsocket_client.so websocket_client.go

# 编译C++示例
if command -v g++ &> /dev/null; then
    echo "Building C++ example..."
    g++ -std=c++11 -Wall -O2 -o example example.cpp -L. -lwebsocket_client
    echo "C++ example built successfully"
else
    echo "Warning: g++ not found, skipping C++ example"
fi

echo "Build completed successfully!"
echo ""
echo "Usage:"
echo "  Go:  go run main.go"
echo "  C++: LD_LIBRARY_PATH=. ./example"
