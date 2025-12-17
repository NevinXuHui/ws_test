# WebSocket 设备连接客户端

## 项目说明
基于Go语言实现的设备WebSocket连接客户端，支持C++调用和ROS2集成。

## 功能特性
- HTTP请求获取WebSocket连接地址
- 一机一密校验（SHA-256 + Base64签名）
- WebSocket长连接与自动心跳（10秒间隔）
- C库接口，支持C++调用
- 消息接收和断开连接回调
- 日志文件输出（可配置路径和控制台输出）
- 多架构支持（x86_64/aarch64）

## 目录结构
```
├── go/                      # Go源码目录
│   ├── main.go              # Go独立运行版本
│   ├── websocket_client.go  # C库封装版本
│   ├── build.sh             # Go库编译脚本
│   ├── go.mod
│   └── go.sum
├── cpp/                     # C++源码目录
│   ├── example/             # 示例代码
│   │   └── example.cpp
│   ├── include/             # 头文件
│   │   ├── websocket_client.h
│   │   └── libwebsocket_client.h
│   ├── lib/<arch>/          # 库文件（按架构区分）
│   │   └── libwebsocket_client.so
│   ├── bin/<arch>/          # 可执行文件（按架构区分）
│   │   └── example
│   ├── config.json          # 配置文件
│   └── Makefile
├── ros2_ws/                 # ROS2工作空间
│   ├── src/
│   │   ├── websocket_client_ros/
│   │   │   ├── src/websocket_node.cpp
│   │   │   ├── include/
│   │   │   ├── lib/<arch>/
│   │   │   ├── config/config.json
│   │   │   ├── launch/websocket.launch.py
│   │   │   ├── CMakeLists.txt
│   │   │   └── package.xml
│   │   └── homi_speech_interface/
│   ├── test/                # 测试脚本
│   ├── build_ros.sh         # ROS2编译脚本
│   └── run_node.sh          # 运行脚本
├── docs/                    # 文档目录
├── build.sh                 # 全量构建脚本
└── run.sh                   # 运行脚本
```

## 使用方法

### 全量构建
```bash
./build.sh
```

### Go程序直接运行
```bash
cd go
go run main.go
```

### C++程序调用
```bash
cd cpp
make
make run
```

### ROS2节点
```bash
# 编译ROS2
cd ros2_ws
./build_ros.sh

# 运行节点
./run_node.sh
```

### ROS2话题和服务
- `/homi_speech/sigc_event_topic` (homi_speech_interface/SIGCEvent): 接收WebSocket消息
- `/homi_speech/sigc_data_service` (homi_speech_interface/SIGCData): 发送WebSocket消息
- `/homi_speech/speech_net_reconnect_service` (std_srvs/Trigger): 重连服务

## C++ API接口
```cpp
void SetMessageCallback(MessageCallback cb);
void SetDisconnectCallback(DisconnectCallback cb);
int ConnectWithConfig(const char* configPath);
void Disconnect();
int IsConnected();
int SendMsg(const char* message);
```

## 配置文件 (config.json)
```json
{
    "apiUrl": "https://...",
    "deviceId": "...",
    "macId": "...",
    "sn": "...",
    "cmei": "...",
    "deviceType": "...",
    "firmwareVersion": "...",
    "softwareVersion": "...",
    "sdkVersion": "...",
    "innerType": "...",
    "snPwd": "...",
    "applicationVersion": "1.0.7",
    "logPath": "/mine/ws_test_log",
    "logToConsole": true,
    "otaVersions": [...],
    "extraParams": {},
    "capabilityParams": {}
}
```

### 日志配置说明
- `logPath`: 日志目录路径，为空则只输出到控制台
- `logToConsole`: 是否同时输出到控制台（默认true）

日志文件自动按日期命名：`websocket_2025-12-17.log`

## 依赖
- Go 1.21+
- github.com/gorilla/websocket v1.5.1
- ROS2 (humble/iron/jazzy)
