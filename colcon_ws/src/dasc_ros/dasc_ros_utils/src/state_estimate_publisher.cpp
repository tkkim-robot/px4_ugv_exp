// this file receives the state estimate from the px4 and republishes it

#include <memory>

#include "dasc_msgs/msg/state.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class StatePublisher : public rclcpp::Node {
public:
  StatePublisher() : Node("px4_state_publisher") {

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5),
                           qos_profile);

    subscription_ =
        this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "fmu/out/vehicle_local_position", qos,
            std::bind(&StatePublisher::state_callback, this, _1));
  }

private:
  void topic_callback(const px4_msgs::msg::VehicleLocalPosition &msg) const {

    // check validity
    if (!msg->xy_valid) {
      RCLCPP_WARN(this->get_logger(), "Rejecting: xy_valid=false");
      return;
    }
    if (!msg->z_valid) {
      RCLCPP_WARN(this->get_logger(), "Rejecting: z_valid=false");
      return;
    }

    if (!msg->v_xy_valid) {
      RCLCPP_WARN(this->get_logger(), "Rejecting: v_xy_valid=false");
      return;
    }
    if (!msg->v_z_valid) {
      RCLCPP_WARN(this->get_logger(), "Rejecting: v_z_valid=false");
      return;
    }
    if (!msg->heading_good_for_control) {
      RCLCPP_WARN(this->get_logger(),
                  "Rejecting: heading_good_for_control=false");
      return;
    }

    // passed all the tests

    dasc_msgs::msg::State state_msg;
    state_msg.header.frame_id = "px4_world/NED";
    state_msg.header.stamp = this->get_clock().now();
    state_msg.timestamp = msg->timestamp;
    state_msg.timstamp_sample = msg->timestamp_sample;

    state_msg.pose.position.x = i
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StatePublisher>());
  rclcpp::shutdown();
  return 0;
}
