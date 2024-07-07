#include "geometry_msgs/msg/transform_stamped.hpp"
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <chrono>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class ViconPX4Bridge : public rclcpp::Node {
public:
  ViconPX4Bridge() : Node("vicon_px4_bridge") {
    this->declare_parameter<std::string>("px4_name", "drone1");
    this->declare_parameter<std::string>("vicon_name", "drone1");
    std::string vicon_name;
    this->get_parameter("px4_name", px4_name);
    this->get_parameter("vicon_name", vicon_name);
    std::string vicon_sub_name = "vicon/" + vicon_name + "/" + vicon_name;

    std::string timesync_sub_name = px4_name + "/fmu/out/timesync";
    std::string px4_pub_name = px4_name + "/fmu/in/vehicle_visual_odometry";

    RCLCPP_INFO(this->get_logger(), "vicon_sub_name: %s \n",
                vicon_sub_name.c_str());
    RCLCPP_INFO(this->get_logger(), "timesync_sub_name: %s \n",
                timesync_sub_name.c_str());
    RCLCPP_INFO(this->get_logger(), "px4_pub_name: %s \n",
                px4_pub_name.c_str());
    RCLCPP_INFO(this->get_logger(), "Using Vicon -> Visual Odometry callback!");

    position_sub_ =
        this->create_subscription<geometry_msgs::msg::TransformStamped>(
            vicon_sub_name, 1,
            std::bind(&ViconPX4Bridge::position_callback, this, _1));

    odometry_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
        px4_pub_name, 10);



    // timesync_sub_ = this->create_subscription<px4_msgs::msg::TimesyncStatus>(
    //     timesync_sub_name, 10,
    //     [this](const px4_msgs::msg::TimesyncStatus::UniquePtr msg) {
    //       this->px4_timestamp_.store(msg->timestamp);
    //       this->px4_server_timestamp_.store(
    //           this->get_clock()->now().nanoseconds());
    //     });
  }

private:
  std::string px4_name;

  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_pub_;

  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr
      position_sub_;
  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
  std::atomic<uint64_t> px4_timestamp_;
  std::atomic<uint64_t> px4_server_timestamp_;
   
  uint8_t counter_ = 0;

  uint64_t get_current_timestamp() {
    auto delta =
        (this->get_clock()->now().nanoseconds() - this->px4_server_timestamp_) /
        1e6;
    return px4_timestamp_.load() + delta;
  }

  void
  position_callback(const geometry_msgs::msg::TransformStamped &vicon_msg) {

    auto message = px4_msgs::msg::VehicleOdometry();
    constexpr int downsample=2;
    counter_++;
    if (!(counter_ % downsample == 0))
    {
	    return;
    }

    // // timestamps
    // auto timestamp = get_current_timestamp();
    // message.timestamp = timestamp;
    // message.timestamp_sample = timestamp;
    //
    message.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    message.timestamp_sample = message.timestamp;

    // pose frame
    message.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

    // position
    message.position[0] = vicon_msg.transform.translation.y;
    message.position[1] = vicon_msg.transform.translation.x;
    message.position[2] = -vicon_msg.transform.translation.z;

    // quaternion
    auto q_vicon = vicon_msg.transform.rotation;
    tf2::Quaternion tf2_vicon(q_vicon.x, q_vicon.y, q_vicon.z, q_vicon.w);
    auto yaw = tf2::getYaw(tf2_vicon);
    auto yaw_ned = -yaw + 1.57;
    Eigen::Quaternion<double> out(cos(yaw_ned / 2.0), 0, 0, sin(yaw_ned / 2.0));

    message.q[0] = out.w();
    message.q[1] = out.x();
    message.q[2] = out.y();
    message.q[3] = out.z();

    // velocity frame
    message.velocity_frame =
        px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;

    for (size_t i = 0; i < 3; i++) {

      // velocity
      message.velocity[i] = NAN;

      // angular velocity
      message.angular_velocity[i] = NAN;
    }

    // position covariance
    for (size_t i = 0; i < 3; i++) {
      message.position_variance[i] = 0.01;
      message.orientation_variance[i] = 0.0523; // approx 3 degrees
      message.velocity_variance[i] = NAN;
    }

    message.reset_counter = 0;

    odometry_pub_->publish(message);
  }
};

int main(int argc, char *argv[]) {
  std::cout << "Starting vicon px4 bridge node..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ViconPX4Bridge>());

  rclcpp::shutdown();
  return 0;
}
