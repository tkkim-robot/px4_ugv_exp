cmake_minimum_required(VERSION 3.8)
project(dasc_ros_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dasc_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)


if(TARGET tf2_geometry_msgs::tf2_geometry_msgs)
  get_target_property(_include_dirs tf2_geometry_msgs::tf2_geometry_msgs INTERFACE_INCLUDE_DIRECTORIES)
else()
  set(_include_dirs ${tf2_geometry_msgs_INCLUDE_DIRS})
endif()

find_file(TF2_CPP_HEADERS
  NAMES tf2_geometry_msgs.hpp
  PATHS ${_include_dirs}
  NO_CACHE
  PATH_SUFFIXES tf2_geometry_msgs
)

add_executable(setpointPublisher src/setpoint_publisher.cpp)
ament_target_dependencies(
	setpointPublisher
  geometry_msgs
  message_filters
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  dasc_msgs
  px4_msgs
)

if(EXISTS ${TF2_CPP_HEADERS})
  target_compile_definitions(setpointPublisher PUBLIC -DTF2_CPP_HEADERS)
endif()


install(TARGETS
	setpointPublisher
  DESTINATION lib/${PROJECT_NAME})


## install python executables
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/publish_lab_poly.py
  scripts/publish_setpoint.py
  DESTINATION lib/${PROJECT_NAME}
  )


ament_package()
