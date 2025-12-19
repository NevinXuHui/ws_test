#!/bin/bash
cd "$(dirname "$0")"

ARCH=$(uname -m)
OUT_DIR="../cpp/lib/${ARCH}"
mkdir -p "$OUT_DIR"

CGO_ENABLED=1 go build -buildmode=c-shared -o "${OUT_DIR}/libwebtransport_client.so" webtransport_client.go

# 删除自动生成的头文件，使用手写的
rm -f "${OUT_DIR}/libwebtransport_client.h"

echo "Built: ${OUT_DIR}/libwebtransport_client.so"
