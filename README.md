# 设备连接客户端

基于Go语言实现的设备连接客户端，支持WebSocket和WebTransport两种协议，提供C++调用和ROS2集成。

## 目录结构
```
├── websocket/               # WebSocket方案
│   ├── go/                  # Go源码
│   ├── cpp/                 # C++调用
│   ├── ros2_ws/             # ROS2工作空间
│   ├── build.sh             # 全量编译
│   └── run.sh
├── webtransport/            # WebTransport方案
│   ├── client/              # 客户端
│   ├── server/              # 测试服务端
│   ├── clib/                # C库封装
│   ├── cpp/                 # C++调用
│   ├── ros2_ws/             # ROS2工作空间
│   ├── build.sh             # 全量编译
│   └── run.sh
└── docs/                    # 文档
```

## WebSocket

基于gorilla/websocket，支持一机一密校验和自动心跳。

```bash
cd websocket

# 全量编译
./build.sh

# 运行
./run.sh              # Go程序
cd ros2_ws && ./run_node.sh   # ROS2节点
```

### ROS2接口
- `/homi_speech/sigc_event_topic` (SIGCEvent): 接收消息
- `/homi_speech/sigc_data_service` (SIGCData): 发送消息
- `/homi_speech/speech_net_reconnect_service` (Trigger): 重连

## WebTransport

基于HTTP/3 + QUIC，支持可靠/不可靠传输。

```bash
cd webtransport

# 全量编译
./build.sh

# 运行
./run.sh server              # 服务端
./run.sh client [local|remote]   # 客户端

# ROS2节点
cd ros2_ws && ./run_node.sh

# 测试脚本
./ros2_ws/reconnect.sh
./ros2_ws/send_reliable.sh
./ros2_ws/send_unreliable.sh
./ros2_ws/subscribe.sh
```

### ROS2接口
- `/webtransport/reliable_recv` (String): 接收可靠消息
- `/webtransport/unreliable_recv` (String): 接收不可靠消息
- `/webtransport/send_reliable` (Trigger): 发送可靠消息
- `/webtransport/send_unreliable` (Trigger): 发送不可靠消息
- `/webtransport/reconnect` (Trigger): 重连

## 多架构支持
- x86_64
- aarch64

## 依赖
- Go 1.21+
- ROS2 (humble/iron/jazzy)
