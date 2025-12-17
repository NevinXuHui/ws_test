#ifndef WEBSOCKET_CLIENT_H
#define WEBSOCKET_CLIENT_H

#ifdef __cplusplus
extern "C" {
#endif

// 回调函数类型
typedef void (*MessageCallback)(const char* message);
typedef void (*DisconnectCallback)();

// 设置消息接收回调
void SetMessageCallback(MessageCallback cb);

// 设置断开连接回调
void SetDisconnectCallback(DisconnectCallback cb);

// 使用配置文件连接
int ConnectWithConfig(const char* configPath);

// 连接
int Connect(const char* deviceId, const char* macId, const char* sn, const char* cmei,
            const char* deviceType, const char* firmwareVersion, const char* softwareVersion,
            const char* sdkVersion, const char* innerType, const char* snPwd);

// 断开连接
void Disconnect();

// 检查连接状态
int IsConnected();

// 发送消息
int SendMsg(const char* message);

#ifdef __cplusplus
}
#endif

#endif
