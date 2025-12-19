# WebSocket ROS2 节点

WebSocket客户端的ROS2封装，提供话题和服务接口。

## 编译运行

```bash
# 编译
./build_ros.sh

# 运行节点
./run_node.sh
```

## 节点信息

节点名: `websocket_node`

### 参数
| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| config_path | string | config.json | 配置文件路径 |

### 话题
| 话题名 | 类型 | 方向 | 说明 |
|--------|------|------|------|
| /homi_speech/sigc_event_topic | homi_speech_interface/SIGCEvent | 发布 | 接收到的WebSocket消息 |

### 服务
| 服务名 | 类型 | 说明 |
|--------|------|------|
| /homi_speech/sigc_data_service | homi_speech_interface/SIGCData | 发送消息 |
| /homi_speech/speech_net_reconnect_service | std_srvs/Trigger | 手动重连 |

## 消息定义

### SIGCEvent.msg
```
string event
```

### SIGCData.srv
```
string data
---
int32 error_code
```

## 测试脚本

```bash
# 订阅消息
./test/subscribe_sigc.sh

# 监听所有消息
./test/listen_messages.sh
```

## 功能特性

- 自动重连：断开后自动尝试重连
- 定时重连：连接失败后每5秒重试
- 手动重连：通过服务触发重连
- 日志输出：带时间戳的详细日志
