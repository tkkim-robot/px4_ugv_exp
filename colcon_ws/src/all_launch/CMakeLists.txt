cmake_minimum_required(VERSION 3.5)
project(all_launch)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vslam_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(reset_vslam_server src/reset_vslam_server.cpp)

ament_target_dependencies(reset_vslam_server rclcpp geometry_msgs tf2_ros
                          vslam_interfaces Eigen3)

install(TARGETS reset_vslam_server DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
