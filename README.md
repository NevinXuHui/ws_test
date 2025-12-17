# WebSocket 设备连接客户端

## 项目说明
基于Go语言实现的设备WebSocket连接客户端，支持C++调用。

## 功能特性
- HTTP请求获取WebSocket连接地址
- 一机一密校验（SHA-256 + Base64签名）
- WebSocket长连接
- 自动心跳维持（10秒间隔）
- C库接口，支持C++调用
- 消息接收和断开连接回调

## 目录结构
```
├── go/                      # Go源码目录
│   ├── main.go              # Go独立运行版本
│   ├── websocket_client.go  # C库封装版本
│   ├── go.mod
│   └── go.sum
├── cpp/                     # C++源码目录
│   ├── example.cpp          # C++使用示例
│   ├── websocket_client.h   # C头文件
│   ├── config.json          # 配置文件
│   ├── libwebsocket_client.so  # 编译后的库文件
│   └── example              # 编译后的可执行文件
├── ros2/                    # ROS2节点
│   ├── src/websocket_node.cpp
│   ├── launch/websocket.launch.py
│   ├── CMakeLists.txt
│   └── package.xml
├── docs/                    # 文档目录
├── build.sh                 # 构建脚本
├── run.sh                   # 运行脚本
└── README.md
```

## 使用方法

### Go程序直接运行
```bash
cd go
go run main.go
```

### C++程序调用
```bash
# 构建
./build.sh

# 运行
./run.sh
```

### ROS2节点
```bash
# 先构建Go库
./build.sh

# 构建ROS2包
cd ros2
colcon build

# 运行节点
source install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/../cpp
ros2 launch websocket_client_ros websocket.launch.py
```

### ROS2话题
- `websocket/recv` (std_msgs/String): 接收WebSocket消息
- `websocket/send` (std_msgs/String): 发送WebSocket消息

## C++ API接口
```cpp
// 设置回调
void SetMessageCallback(MessageCallback cb);
void SetDisconnectCallback(DisconnectCallback cb);

// 连接
int ConnectWithConfig(const char* configPath);
int Connect(deviceId, macId, sn, cmei, ...);

// 断开连接
void Disconnect();

// 检查连接状态
int IsConnected();

// 发送消息
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
    "otaVersions": [...],
    "extraParams": {},
    "capabilityParams": {}
}
```

## 依赖
- Go 1.21+
- github.com/gorilla/websocket v1.5.1
