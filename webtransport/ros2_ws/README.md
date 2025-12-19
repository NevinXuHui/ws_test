# WebTransport ROS2 节点

WebTransport客户端的ROS2封装，支持可靠和不可靠两种传输模式。

## 编译运行

```bash
# 编译
./build_ros.sh

# 运行节点
./run_node.sh
```

## 节点信息

节点名: `webtransport_node`

### 参数
| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| config_path | string | config.json | 配置文件路径 |

### 话题
| 话题名 | 类型 | 方向 | 说明 |
|--------|------|------|------|
| /webtransport/reliable_recv | std_msgs/String | 发布 | 接收的可靠消息(Stream) |
| /webtransport/unreliable_recv | std_msgs/String | 发布 | 接收的不可靠消息(Datagram) |

### 服务
| 服务名 | 类型 | 说明 |
|--------|------|------|
| /webtransport/send_reliable | std_srvs/Trigger | 发送可靠消息 |
| /webtransport/send_unreliable | std_srvs/Trigger | 发送不可靠消息 |
| /webtransport/reconnect | std_srvs/Trigger | 手动重连 |

## 测试脚本

```bash
# 重连
./reconnect.sh

# 发送可靠消息
./send_reliable.sh

# 发送不可靠消息
./send_unreliable.sh

# 订阅接收消息
./subscribe.sh
```

## 功能特性

- 可靠传输：基于QUIC Stream，保证消息顺序和可靠性
- 不可靠传输：基于QUIC Datagram，低延迟但不保证送达
- 自动重连：断开后自动尝试重连
- 定时重连：连接失败后每5秒重试
