#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "websocket_client.h"

class WebSocketNode : public rclcpp::Node {
public:
    WebSocketNode() : Node("websocket_node") {
        // 声明参数
        this->declare_parameter<std::string>("config_path", "config.json");
        
        // 创建发布者 - 发布收到的WebSocket消息
        msg_pub_ = this->create_publisher<std_msgs::msg::String>("websocket/recv", 10);
        
        // 创建订阅者 - 订阅要发送的消息
        msg_sub_ = this->create_subscription<std_msgs::msg::String>(
            "websocket/send", 10,
            std::bind(&WebSocketNode::sendCallback, this, std::placeholders::_1));
        
        // 设置回调
        SetMessageCallback(onMessage);
        SetDisconnectCallback(onDisconnect);
        
        // 保存节点实例
        instance_ = this;
        
        // 连接WebSocket
        std::string config_path = this->get_parameter("config_path").as_string();
        RCLCPP_INFO(this->get_logger(), "加载配置: %s", config_path.c_str());
        
        int ret = ConnectWithConfig(config_path.c_str());
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "WebSocket连接失败: %d", ret);
        } else {
            RCLCPP_INFO(this->get_logger(), "WebSocket连接成功");
        }
        
        // 定时检查连接状态
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&WebSocketNode::checkConnection, this));
    }
    
    ~WebSocketNode() {
        Disconnect();
        RCLCPP_INFO(this->get_logger(), "WebSocket已断开");
    }

private:
    static WebSocketNode* instance_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr msg_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr msg_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    static void onMessage(const char* message) {
        if (instance_ && instance_->msg_pub_) {
            auto msg = std_msgs::msg::String();
            msg.data = message;
            instance_->msg_pub_->publish(msg);
            RCLCPP_DEBUG(instance_->get_logger(), "发布消息: %s", message);
        }
    }
    
    static void onDisconnect() {
        if (instance_) {
            RCLCPP_WARN(instance_->get_logger(), "WebSocket连接断开!");
        }
    }
    
    void sendCallback(const std_msgs::msg::String::SharedPtr msg) {
        if (IsConnected()) {
            SendMsg(msg->data.c_str());
            RCLCPP_DEBUG(this->get_logger(), "发送消息: %s", msg->data.c_str());
        }
    }
    
    void checkConnection() {
        if (!IsConnected()) {
            RCLCPP_WARN(this->get_logger(), "连接已断开，尝试重连...");
            std::string config_path = this->get_parameter("config_path").as_string();
            ConnectWithConfig(config_path.c_str());
        }
    }
};

WebSocketNode* WebSocketNode::instance_ = nullptr;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WebSocketNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
