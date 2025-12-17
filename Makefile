# Makefile for WebSocket Client Library

# Go编译器
GO = go
# C++编译器
CXX = g++

# 库名称
LIB_NAME = libwebsocket_client
SHARED_LIB = $(LIB_NAME).so
STATIC_LIB = $(LIB_NAME).a

# 编译标志
CXXFLAGS = -std=c++11 -Wall -O2
LDFLAGS = -L. -lwebsocket_client

.PHONY: all clean shared static example

all: shared

# 编译共享库
shared:
	$(GO) build -buildmode=c-shared -o $(SHARED_LIB) websocket_client.go

# 编译静态库  
static:
	$(GO) build -buildmode=c-archive -o $(STATIC_LIB) websocket_client.go

# 编译C++示例
example: shared
	$(CXX) $(CXXFLAGS) -o example example.cpp $(LDFLAGS)

# 运行示例
run: example
	LD_LIBRARY_PATH=. ./example

# 清理
clean:
	rm -f $(SHARED_LIB) $(STATIC_LIB) *.h example websocket_client.h

# 安装依赖
deps:
	$(GO) mod tidy

# 帮助
help:
	@echo "Available targets:"
	@echo "  shared  - Build shared library (.so)"
	@echo "  static  - Build static library (.a)" 
	@echo "  example - Build C++ example"
	@echo "  run     - Build and run example"
	@echo "  clean   - Clean build files"
	@echo "  deps    - Install Go dependencies"
