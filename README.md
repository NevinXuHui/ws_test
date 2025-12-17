# WebSocket 设备连接客户端

## 项目说明
基于Go语言实现的设备WebSocket连接客户端，支持C++调用。

## 功能特性
- HTTP请求获取WebSocket连接地址
- 一机一密校验（SHA-256 + Base64签名）
- WebSocket长连接
- 自动心跳维持（10秒间隔）
- C库接口，支持C++调用

## 文件结构
```
├── main.go              # Go独立运行版本
├── websocket_client.go  # C库封装版本
├── websocket_client.h   # C头文件
├── example.cpp          # C++使用示例
├── build.sh             # 构建脚本
├── docs/                # 文档目录
└── README.md
```

## 使用方法

### Go程序直接运行
```bash
go run main.go
```

### C++程序调用
```bash
# 构建
./build.sh

# 运行
LD_LIBRARY_PATH=. ./example
```

## C++ API接口
```cpp
// 连接 (返回: 0=成功, -1=获取URL失败, -2=连接失败)
int Connect(deviceId, macId, sn, cmei, deviceType, firmwareVersion, 
            softwareVersion, sdkVersion, innerType, snPwd);

// 断开连接
void Disconnect();

// 检查连接状态 (返回: 1=已连接, 0=未连接)
int IsConnected();

// 发送消息 (返回: 0=成功)
int SendMsg(const char* message);
```

## 依赖
- Go 1.21+
- github.com/gorilla/websocket v1.5.1
