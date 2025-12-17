#include "homi_speech_interface/msg/sigc_event.hpp"
#include "homi_speech_interface/srv/sigc_data.hpp"
#include "websocket_client.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

class WebSocketNode : public rclcpp::Node {
public:
  WebSocketNode() : Node("websocket_node") {
    this->declare_parameter<std::string>("config_path", "config.json");

    // 发布WebSocket接收到的消息
    ws_recv_pub_ =
        this->create_publisher<homi_speech_interface::msg::SIGCEvent>(
            "/homi_speech/sigc_event_topic", 10);

    // 提供WebSocket发送消息的服务
    ws_send_service_ =
        this->create_service<homi_speech_interface::srv::SIGCData>(
            "/homi_speech/sigc_data_service",
            std::bind(&WebSocketNode::sendMessageCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

    // 提供WebSocket重连服务
    ws_reconnect_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/homi_speech/speech_net_reconnect_service",
        std::bind(&WebSocketNode::NetReconnectCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    SetMessageCallback(onMessage);
    SetDisconnectCallback(onDisconnect);
    instance_ = this;

    std::string config_path = this->get_parameter("config_path").as_string();
    RCLCPP_INFO(this->get_logger(), "加载配置: %s", config_path.c_str());

    int ret = ConnectWithConfig(config_path.c_str());
    if (ret != 0) {
      RCLCPP_ERROR(this->get_logger(), "WebSocket连接失败: %d，启动定时重连",
                   ret);
      startReconnectTimer();
    } else {
      RCLCPP_INFO(this->get_logger(), "WebSocket连接成功");
    }
  }

  ~WebSocketNode() {
    Disconnect();
    RCLCPP_INFO(this->get_logger(), "WebSocket已断开");
  }

private:
  static WebSocketNode *instance_;
  rclcpp::Publisher<homi_speech_interface::msg::SIGCEvent>::SharedPtr
      ws_recv_pub_;
  rclcpp::Service<homi_speech_interface::srv::SIGCData>::SharedPtr
      ws_send_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ws_reconnect_service_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;

  void sendMessageCallback(
      const std::shared_ptr<homi_speech_interface::srv::SIGCData::Request>
          request,
      std::shared_ptr<homi_speech_interface::srv::SIGCData::Response>
          response) {
    if (IsConnected()) {
      int ret = SendMsg(request->data.c_str());
      response->error_code = ret;
      RCLCPP_INFO(this->get_logger(), "WebSocket发送: %s, 结果: %d",
                  request->data.c_str(), ret);
    } else {
      response->error_code = -1;
      RCLCPP_WARN(this->get_logger(), "WebSocket未连接，发送失败");
    }
  }

  void
  NetReconnectCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    RCLCPP_INFO(this->get_logger(), "NetReconnectCallback");
    Disconnect();
    std::string config_path = this->get_parameter("config_path").as_string();
    int ret = ConnectWithConfig(config_path.c_str());
    res->success = (ret == 0);
    if (ret == 0) {
      res->message = "重连成功";
      RCLCPP_INFO(this->get_logger(), "手动重连成功");
      if (reconnect_timer_) {
        reconnect_timer_->cancel();
        reconnect_timer_.reset();
      }
    } else {
      res->message = "重连失败: " + std::to_string(ret);
      RCLCPP_ERROR(this->get_logger(), "手动重连失败: %d", ret);
    }
  }

  static void onMessage(const char *message) {
    if (instance_ && instance_->ws_recv_pub_) {
      auto event = homi_speech_interface::msg::SIGCEvent();
      event.event = message;
      instance_->ws_recv_pub_->publish(event);
      RCLCPP_INFO(instance_->get_logger(), "WebSocket接收: %s", message);
    }
  }

  static void onDisconnect() {
    if (instance_) {
      RCLCPP_WARN(instance_->get_logger(), "WebSocket连接断开，尝试重连...");
      std::string config_path =
          instance_->get_parameter("config_path").as_string();
      int ret = ConnectWithConfig(config_path.c_str());
      if (ret == 0) {
        RCLCPP_INFO(instance_->get_logger(), "重连成功");
        if (instance_->reconnect_timer_) {
          instance_->reconnect_timer_->cancel();
          instance_->reconnect_timer_.reset();
        }
      } else {
        RCLCPP_ERROR(instance_->get_logger(), "重连失败: %d，启动定时重连",
                     ret);
        instance_->startReconnectTimer();
      }
    }
  }

  void startReconnectTimer() {
    if (!reconnect_timer_) {
      reconnect_timer_ = this->create_wall_timer(
          std::chrono::seconds(5),
          std::bind(&WebSocketNode::tryReconnect, this));
    }
  }

  void tryReconnect() {
    std::string config_path = this->get_parameter("config_path").as_string();
    int ret = ConnectWithConfig(config_path.c_str());
    if (ret == 0) {
      RCLCPP_INFO(this->get_logger(), "定时重连成功");
      reconnect_timer_->cancel();
      reconnect_timer_.reset();
    }
  }
};

WebSocketNode *WebSocketNode::instance_ = nullptr;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WebSocketNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
