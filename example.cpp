#include <iostream>
#include <thread>
#include <chrono>
#include "websocket_client.h"

int main() {
    // 连接WebSocket
    int result = ConnectWebSocket(
        "1222004229866666660001505",  // deviceId
        "FC23CD911857",               // macId
        "1222004229866666660001505",  // sn
        "866666660001505",            // cmei
        "591884",                     // deviceType
        "1.0.1",                      // firmwareVersion
        "1.0.7",                      // softwareVersion
        "1.0.7",                      // sdkVersion
        "1",                          // innerType
        "14mrrGJH"                    // snPwd
    );
    
    if (result != 0) {
        std::cerr << "Connection failed: " << result << std::endl;
        return -1;
    }
    
    std::cout << "WebSocket connected successfully!" << std::endl;
    
    // 发送测试消息
    SendMessage("{\"test\": \"message from C++\"}");
    
    // 保持连接30秒
    for (int i = 0; i < 30 && IsConnected(); i++) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "Connected for " << (i+1) << " seconds..." << std::endl;
    }
    
    // 断开连接
    Disconnect();
    std::cout << "Disconnected" << std::endl;
    return 0;
}
