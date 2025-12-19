#ifndef WEBTRANSPORT_CLIENT_H
#define WEBTRANSPORT_CLIENT_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*MessageCallback)(const char* message, int reliable);
typedef void (*DisconnectCallback)();

void SetMessageCallback(MessageCallback cb);
void SetDisconnectCallback(DisconnectCallback cb);
int ConnectWithConfig(const char* configPath);
void Disconnect();
int IsConnected();
int SendReliable(const char* message);
int SendUnreliable(const char* message);

#ifdef __cplusplus
}
#endif

#endif
