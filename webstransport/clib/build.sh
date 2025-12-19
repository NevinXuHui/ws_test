#!/bin/bash
cd "$(dirname "$0")"

ARCH=$(uname -m)
OUT_DIR="../cpp/lib/${ARCH}"
mkdir -p "$OUT_DIR"

CGO_ENABLED=1 go build -buildmode=c-shared -o "${OUT_DIR}/libwebtransport_client.so" webtransport_client.go

# 复制头文件
cp "${OUT_DIR}/libwebtransport_client.h" ../cpp/include/ 2>/dev/null || true

echo "Built: ${OUT_DIR}/libwebtransport_client.so"
