# WebSocket 设备连接客户端

## 项目说明
基于Go语言实现的设备WebSocket连接客户端，用于与平台建立长连接并维持心跳。支持封装为C库供C++调用。

## 功能特性
- HTTP请求获取WebSocket连接地址
- 一机一密校验（SHA-256 + Base64签名）
- WebSocket长连接
- 自动心跳维持（10秒间隔）
- 完整的日志记录（精确到毫秒）
- C库接口，支持C++调用

## 使用方法

### Go程序直接运行
1. 修改 `main.go` 中的设备配置参数
2. 运行 `go mod tidy` 安装依赖
3. 执行 `go run main.go`

### C++程序调用
1. 编译Go库：`make shared`
2. 编译C++示例：`make example`
3. 运行示例：`make run`

## C++ API接口

### 连接WebSocket
```cpp
int ConnectWebSocket(
    const char* deviceId,
    const char* macId, 
    const char* sn,
    const char* cmei,
    const char* deviceType,
    const char* firmwareVersion,
    const char* softwareVersion,
    const char* sdkVersion,
    const char* innerType,
    const char* snPwd
);
```

### 设置消息回调
```cpp
void SetMessageCallback(MessageCallback callback);
```

### 发送消息
```cpp
int SendMessage(const char* message);
```

### 检查连接状态
```cpp
int IsConnected();
```

### 断开连接
```cpp
void Disconnect();
```

## 配置参数
- **DeviceID**: 设备ID（使用SN号）
- **MacID**: 设备MAC地址
- **SN**: 设备序列号
- **CMEI**: 设备CMEI
- **DeviceType**: 设备类型
- **FirmwareVersion**: 固件版本
- **SoftwareVersion**: 软件版本
- **SDKVersion**: SDK版本
- **InnerType**: 内部类型
- **SNPwd**: 设备密码

## 依赖包
- `github.com/gorilla/websocket v1.5.1`
