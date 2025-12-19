#ifndef WEBTRANSPORT_CLIENT_H
#define WEBTRANSPORT_CLIENT_H

#ifdef __cplusplus
extern "C" {
#endif

// 回调函数类型: message=消息内容, reliable=1可靠/0不可靠
typedef void (*MessageCallback)(const char* message, int reliable);
typedef void (*DisconnectCallback)();

void SetMessageCallback(MessageCallback cb);
void SetDisconnectCallback(DisconnectCallback cb);
int ConnectWithConfig(const char* configPath);
void Disconnect();
int IsConnected();
int SendReliable(const char* message);    // 可靠传输
int SendUnreliable(const char* message);  // 不可靠传输

#ifdef __cplusplus
}
#endif

#endif
