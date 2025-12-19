# WebTransport Go 示例

基于 `github.com/quic-go/webtransport-go` 实现，支持可靠和不可靠传输，提供 C++ 调用接口。

## 传输方式

| 方式 | API | 特点 | 适用场景 |
|------|-----|------|----------|
| 可靠 (Stream) | `SendReliable` | 按序到达、自动重传 | 文件传输、命令控制 |
| 不可靠 (Datagram) | `SendUnreliable` | 低延迟、可能丢包 | 实时音视频、传感器数据 |

## 目录结构

```
├── server/          # Go 服务器
├── client/          # Go 客户端
├── clib/            # C库封装源码
│   ├── webtransport_client.go
│   └── build.sh
└── cpp/             # C++ 调用
    ├── include/
    ├── lib/<arch>/
    ├── example/
    └── Makefile
```

## 快速开始

```bash
# 生成证书
./gen_cert.sh

# 启动服务器
./run.sh server

# Go 客户端测试
./run.sh client

# C++ 客户端测试
cd cpp && make run
```

## C++ API

```cpp
// 回调 (reliable: 1=可靠, 0=不可靠)
void SetMessageCallback(void (*cb)(const char* msg, int reliable));
void SetDisconnectCallback(void (*cb)());

// 连接
int ConnectWithConfig(const char* configPath);
void Disconnect();
int IsConnected();

// 发送
int SendReliable(const char* message);    // 可靠
int SendUnreliable(const char* message);  // 不可靠
```

## 配置文件 (config.json)

```json
{
    "serverUrl": "https://localhost:4433/webtransport"
}
```

## 依赖

- Go 1.23+
- github.com/quic-go/webtransport-go v0.9.0
