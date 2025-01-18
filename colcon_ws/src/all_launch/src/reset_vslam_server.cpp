#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "vslam_interfaces/srv/reset_vslam.hpp"
#include <Eigen/Dense>

// Function to convert geometry_msgs::msg::Transform to Eigen::Affine3d
Eigen::Affine3d
transformToEigen(const geometry_msgs::msg::Transform &transform) {
  // Extract translation
  Eigen::Vector3d translation(transform.translation.x, transform.translation.y,
                              transform.translation.z);

  // Extract rotation (quaternion to rotation matrix)
  Eigen::Quaterniond quaternion(transform.rotation.w, transform.rotation.x,
                                transform.rotation.y, transform.rotation.z);
  Eigen::Matrix3d rotation = quaternion.toRotationMatrix();

  // Create an Affine3d object
  Eigen::Affine3d eigen_transform =
      Eigen::Translation3d(translation) * rotation;

  return eigen_transform;
}

// Function to convert Eigen::Affine3d to geometry_msgs::msg::Transform
geometry_msgs::msg::Transform
eigenToTransform(const Eigen::Affine3d &eigen_transform) {
  geometry_msgs::msg::Transform transform;

  // Extract translation
  const Eigen::Vector3d translation = eigen_transform.translation();
  transform.translation.x = translation.x();
  transform.translation.y = translation.y();
  transform.translation.z = translation.z();

  // Extract rotation (convert rotation matrix to quaternion)
  Eigen::Quaterniond quaternion(eigen_transform.rotation());
  transform.rotation.w = quaternion.w();
  transform.rotation.x = quaternion.x();
  transform.rotation.y = quaternion.y();
  transform.rotation.z = quaternion.z();

  return transform;
}

class ResetVslamServer : public rclcpp::Node {
public:
  ResetVslamServer() : Node("reset_vslam_server") {

    // create the tf_buffer
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

    // create the tf broadcaster
    tf_broadcaster_ =
        std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    // create the tf listener
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // Create the service
    service_ = this->create_service<vslam_interfaces::srv::ResetVslam>(
        "reset_vslam",
        std::bind(&ResetVslamServer::set_transform_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

private:
  bool get_transform(geometry_msgs::msg::TransformStamped &transform,
                     const std::string &header_frame,
                     const std::string child_frame) {
    try {
      // Lookup the current transform
      transform = tf_buffer_->lookupTransform(header_frame, child_frame,
                                              rclcpp::Time(0));
      return true;
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), e.what());
      return false;
    }
    return false;
  }

  void set_transform_callback(
      const std::shared_ptr<vslam_interfaces::srv::ResetVslam::Request> request,
      std::shared_ptr<vslam_interfaces::srv::ResetVslam::Response> response) {
    RCLCPP_INFO(get_logger(), "got service call request");
    geometry_msgs::msg::TransformStamped transform_vicon, transform_vslam;

    bool success_1 =
        get_transform(transform_vicon, "vicon/world", "vicon/px4_2/px4_2");
    bool success_2 = get_transform(transform_vslam, "map", "base_link");

    if (success_1 && success_2) {
      RCLCPP_DEBUG(get_logger(), "successfully got both transforms");

      // calculate the relative transform
      // convert to eigen
      auto eigen_vicon = transformToEigen(transform_vicon.transform);
      auto eigen_vslam = transformToEigen(transform_vslam.transform);

      // get the relative transform
      auto eigen_relative_transform = eigen_vicon * eigen_vslam.inverse();

      // Create a static transform message
      geometry_msgs::msg::TransformStamped static_transform;
      static_transform.header.stamp = this->get_clock()->now();
      static_transform.header.frame_id = "vicon/world";
      static_transform.child_frame_id = "map";
      static_transform.transform = eigenToTransform(eigen_relative_transform);

      // Publish the transform
      tf_broadcaster_->sendTransform(static_transform);
      RCLCPP_INFO(get_logger(), "sent static transform");

      response->success = true;
    } else {
      response->success = false;
    }
  }

  rclcpp::Service<vslam_interfaces::srv::ResetVslam>::SharedPtr service_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ResetVslamServer>());
  rclcpp::shutdown();
  return 0;
}
