#include <iostream>
#include <thread>
#include <chrono>

extern "C" {
    int SimpleConnect(const char* deviceId, const char* macId, const char* sn, const char* cmei, const char* deviceType, const char* firmwareVersion, const char* softwareVersion, const char* sdkVersion, const char* innerType, const char* snPwd);
    void SimpleDisconnect();
    int IsSimpleConnected();
}

int main() {
    std::cout << "Testing simple WebSocket client..." << std::endl;
    
    int result = SimpleConnect(
        "1222004229866666660001505",
        "FC23CD911857", 
        "1222004229866666660001505",
        "866666660001505",
        "591884",
        "1.0.1",
        "1.0.7", 
        "1.0.7",
        "1",
        "14mrrGJH"
    );
    
    if (result != 0) {
        std::cerr << "Connection failed: " << result << std::endl;
        return -1;
    }
    
    std::cout << "Connected!" << std::endl;
    
    // 运行15秒
    for (int i = 0; i < 15; i++) {
        if (!IsSimpleConnected()) {
            std::cout << "Connection lost" << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "." << std::flush;
    }
    
    std::cout << "\nDisconnecting..." << std::endl;
    SimpleDisconnect();
    return 0;
}
