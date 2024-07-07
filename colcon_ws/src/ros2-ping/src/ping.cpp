// Devansh Agrawal
// Sept 2023

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Ping : public rclcpp::Node {

public:
  Ping() : Node("ping") {

    ping_rate_hz_ =
        this->declare_parameter<double>("ping_rate_hz", ping_rate_hz_);

    subscriber_ = this->create_subscription<builtin_interfaces::msg::Time>(
        "pong", 10, std::bind(&Ping::pong_callback, this, _1));

    ping_publisher_ =
        this->create_publisher<builtin_interfaces::msg::Time>("ping", 10);
    rt_publisher_ = this->create_publisher<builtin_interfaces::msg::Duration>(
        "round_trip", 10);
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / ping_rate_hz_),
        std::bind(&Ping::timer_callback, this));
  }

private:
  builtin_interfaces::msg::Duration to_msg(uint64_t t_ns) {
    // apparently builtin_interfaces::msg::Duration.nanosec must be in [0, 10e9)
    constexpr uint64_t max_ns = 10e9;

    builtin_interfaces::msg::Duration msg;
    msg.sec = t_ns / max_ns;
    msg.nanosec = t_ns % max_ns;

    return msg;
  }
  void pong_callback(const builtin_interfaces::msg::Time &msg) {

    constexpr uint64_t s2ns = 1e9;
    constexpr double ns2ms = 1e-6;

    builtin_interfaces::msg::Time now_ = this->get_clock()->now();

    uint64_t start_ns = uint64_t(msg.sec) * s2ns + uint64_t(msg.nanosec);
    uint64_t end_ns = uint64_t(now_.sec) * s2ns + uint64_t(now_.nanosec);

    if (end_ns > start_ns) {
      uint64_t round_trip_ns = end_ns - start_ns;
      double round_trip_ms = ns2ms * (double)round_trip_ns;

      rt_publisher_->publish(to_msg(round_trip_ns));

      RCLCPP_INFO(get_logger(), " ...round trip delay: %f ms", round_trip_ms);
    }
  }

  void timer_callback() {

    // generate the ping pulse
    builtin_interfaces::msg::Time msg = this->get_clock()->now();
    ping_publisher_->publish(msg);
    RCLCPP_INFO(get_logger(), "sending ping...");
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr ping_publisher_;
  rclcpp::Publisher<builtin_interfaces::msg::Duration>::SharedPtr rt_publisher_;
  rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr subscriber_;
  double ping_rate_hz_ = 50.0;
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<Ping>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
