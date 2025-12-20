# WebSocket 快速断联检测与重连方案

> 基于 Boost.Beast 实现，针对 5G/WiFi 网络切换场景优化

## 1. 设计目标

| 目标 | 指标 |
|------|------|
| 断联检测响应 | < 100ms |
| 重连启动延迟 | 0ms（立即触发） |
| 适用场景 | 5G/WiFi 网络无缝切换 |

## 2. 多层检测策略

采用并行多层检测机制，确保任一层级检测到断联即可触发重连：

| 层级 | 检测方法 | 响应速度 | 说明 |
|:----:|----------|:--------:|------|
| L1 | 读写错误捕获 | 即时 | 捕获 async_read/write 错误 |
| L2 | TCP Keepalive | 1-2s | 系统级 TCP 探测 |
| L3 | 应用层心跳 | 3-5s | WebSocket Ping/Pong |
| L4 | Netlink 监听 | 即时 | 监控网卡/IP 状态变化 |

## 3. 系统架构

### 3.1 核心组件

- **WebSocketClient**：WebSocket 客户端主类
  - 负责连接建立、数据收发
  - 管理心跳定时器和重连定时器
  - 处理读写错误并触发重连

- **NetworkMonitor**：网络状态监控器
  - 通过 Netlink 监听网卡和 IP 地址变化
  - 检测到网络变化时通知 WebSocketClient

- **ConnectionManager**：连接管理器
  - 统一管理断联检测和重连逻辑
  - 实现指数退避重连策略

### 3.2 组件交互流程

1. WebSocketClient 建立连接后启动心跳和 Keepalive
2. NetworkMonitor 持续监听系统网络状态
3. 任一检测层发现断联 → 通知 ConnectionManager
4. ConnectionManager 关闭旧连接并立即发起重连
5. 重连失败则按指数退避策略重试

## 4. 检测机制详解

### 4.1 TCP Keepalive 激进配置

| 参数 | 值 | 说明 |
|------|:--:|------|
| SO_KEEPALIVE | 1 | 启用 Keepalive |
| TCP_KEEPIDLE | 1s | 连接空闲 1 秒后开始探测 |
| TCP_KEEPINTVL | 1s | 探测包发送间隔 |
| TCP_KEEPCNT | 2 | 2 次失败即判定断联 |

总断联检测时间：最快 3 秒（1s 空闲 + 2×1s 探测）

### 4.2 Netlink 网络监听（Linux）

监听的事件组：
- **RTMGRP_LINK**：网卡状态变化（up/down）
- **RTMGRP_IPV4_IFADDR**：IPv4 地址变化

触发的 Netlink 消息类型：
- RTM_NEWLINK / RTM_DELLINK：链路添加/删除
- RTM_NEWADDR / RTM_DELADDR：地址添加/删除

### 4.3 应用层心跳

- 发送 WebSocket Ping 帧
- 等待 Pong 响应
- 超时未收到则判定断联

## 5. 重连策略

### 5.1 立即重连

检测到断联后延迟 0ms 立即重连，最大化重连速度。

### 5.2 指数退避

连续失败时采用指数退避：

| 重试次数 | 等待时间 |
|:--------:|:--------:|
| 1 | 1s |
| 2 | 2s |
| 3 | 4s |
| 4 | 8s |
| 5 | 16s |
| 6+ | 30s（上限） |

### 5.3 重试上限

- 最大重试次数：10 次
- 达到上限后停止重连，上报错误

## 6. 配置参数汇总

### 6.1 TCP 层

| 参数 | 推荐值 | 说明 |
|------|:------:|------|
| TCP_KEEPIDLE | 1s | 空闲后开始探测时间 |
| TCP_KEEPINTVL | 1s | 探测间隔 |
| TCP_KEEPCNT | 2 | 失败次数阈值 |

### 6.2 应用层

| 参数 | 推荐值 | 说明 |
|------|:------:|------|
| 心跳间隔 | 5s | Ping 发送频率 |
| 心跳超时 | 3s | Pong 响应超时 |
| 重连延迟 | 0s | 立即重连 |
| 最大重试 | 10 | 连续失败上限 |
| 最大退避 | 30s | 退避时间上限 |

## 7. 环境依赖

| 依赖项 | 版本要求 | 用途 |
|--------|:--------:|------|
| Boost | ≥ 1.70 | Beast (WebSocket), Asio (异步 IO) |
| OpenSSL | - | WSS 加密连接（可选） |
| C++ 标准 | ≥ C++14 | 语言特性 |
| Linux | - | Netlink 网络监听 |

## 8. 注意事项

1. **平台兼容性**：Netlink 仅限 Linux，其他平台需替代方案（如 macOS 的 SystemConfiguration）
2. **网络流量**：激进的 Keepalive 配置会增加网络流量，需根据场景调整
3. **退避上限**：建议设置最大退避时间，避免用户等待过长
4. **线程安全**：确保回调在正确的线程/strand 中执行
5. **资源清理**：重连前确保旧连接资源完全释放
