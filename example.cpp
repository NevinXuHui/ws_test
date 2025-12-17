#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include "websocket_client.h"

std::atomic<bool> running(true);

void onMessage(const char* message) {
    std::cout << "[C++回调] 收到消息: " << message << std::endl;
}

void onDisconnect() {
    std::cout << "[C++回调] 连接已断开!" << std::endl;
    running = false;
}

int main() {
    std::cout << "连接WebSocket..." << std::endl;
    
    // 设置回调
    SetMessageCallback(onMessage);
    SetDisconnectCallback(onDisconnect);
    
    // 使用配置文件连接
    int ret = ConnectWithConfig("config.json");
    if (ret != 0) {
        std::cerr << "连接失败: " << ret << std::endl;
        return -1;
    }
    
    // 运行直到断开或超时30秒
    for (int i = 0; i < 300 && running && IsConnected(); i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    Disconnect();
    std::cout << "程序结束" << std::endl;
    return 0;
}
