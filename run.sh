#!/bin/bash

cd "$(dirname "$0")"

if [ ! -f "example" ] || [ ! -f "libwebsocket_client.so" ]; then
    echo "未找到编译文件，先执行构建..."
    ./build.sh
fi

echo "=== 运行WebSocket客户端 ==="
LD_LIBRARY_PATH=. ./example
