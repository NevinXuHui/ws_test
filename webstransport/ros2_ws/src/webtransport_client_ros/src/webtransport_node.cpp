#include "webtransport_client.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

class WebTransportNode : public rclcpp::Node {
public:
  WebTransportNode() : Node("webtransport_node") {
    this->declare_parameter<std::string>("config_path", "config.json");

    reliable_pub_ = this->create_publisher<std_msgs::msg::String>("/webtransport/reliable_recv", 10);
    unreliable_pub_ = this->create_publisher<std_msgs::msg::String>("/webtransport/unreliable_recv", 10);

    reliable_send_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/webtransport/send_reliable",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          (void)req;
          res->success = (SendReliable("test reliable") == 0);
        });

    unreliable_send_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/webtransport/send_unreliable",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          (void)req;
          res->success = (SendUnreliable("test unreliable") == 0);
        });

    reconnect_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/webtransport/reconnect",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          Disconnect();
          res->success = (ConnectWithConfig(config_path_.c_str()) == 0);
        });

    SetMessageCallback(onMessage);
    SetDisconnectCallback(onDisconnect);
    instance_ = this;

    config_path_ = this->get_parameter("config_path").as_string();
    RCLCPP_INFO(this->get_logger(), "config: %s", config_path_.c_str());

    if (ConnectWithConfig(config_path_.c_str()) != 0) {
      RCLCPP_ERROR(this->get_logger(), "connect failed, start reconnect timer");
      startReconnectTimer();
    }
  }

  ~WebTransportNode() { Disconnect(); }

private:
  static WebTransportNode *instance_;
  std::string config_path_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reliable_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr unreliable_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reliable_send_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr unreliable_send_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reconnect_srv_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;

  static void onMessage(const char *msg, int reliable) {
    if (!instance_) return;
    auto m = std_msgs::msg::String();
    m.data = msg;
    if (reliable) {
      instance_->reliable_pub_->publish(m);
      RCLCPP_INFO(instance_->get_logger(), "[Reliable] %s", msg);
    } else {
      instance_->unreliable_pub_->publish(m);
      RCLCPP_INFO(instance_->get_logger(), "[Unreliable] %s", msg);
    }
  }

  static void onDisconnect() {
    if (!instance_) return;
    RCLCPP_WARN(instance_->get_logger(), "disconnected, reconnecting...");
    if (ConnectWithConfig(instance_->config_path_.c_str()) != 0) {
      instance_->startReconnectTimer();
    }
  }

  void startReconnectTimer() {
    if (!reconnect_timer_) {
      reconnect_timer_ = this->create_wall_timer(
          std::chrono::seconds(5), [this]() {
            if (ConnectWithConfig(config_path_.c_str()) == 0) {
              RCLCPP_INFO(this->get_logger(), "reconnected");
              reconnect_timer_->cancel();
              reconnect_timer_.reset();
            }
          });
    }
  }
};

WebTransportNode *WebTransportNode::instance_ = nullptr;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebTransportNode>());
  rclcpp::shutdown();
  return 0;
}
