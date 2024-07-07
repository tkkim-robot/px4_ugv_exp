// Devansh Agrawal
// Sept 2023

#include "rclcpp/rclcpp.hpp"
#include <chrono>

#include "builtin_interfaces/msg/time.hpp"

using std::placeholders::_1;

class Pong : public rclcpp::Node {

public:
  Pong() : Node("pong") {

    subscriber_ = this->create_subscription<builtin_interfaces::msg::Time>(
        "ping", 10, std::bind(&Pong::ping_callback, this, _1));

    publisher_ =
        this->create_publisher<builtin_interfaces::msg::Time>("pong", 10);

    RCLCPP_INFO(get_logger(), "waiting for ping msg...");
  }

private:
  void ping_callback(const builtin_interfaces::msg::Time &msg) {

    // reflect it back!
    publisher_->publish(msg);
    RCLCPP_INFO(get_logger(), "got ping!");
  }

  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr publisher_;
  rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr subscriber_;

}; // class Pong

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<Pong>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
