cmake_minimum_required(VERSION 3.8)
project(ping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)


add_executable(ping src/ping.cpp)
ament_target_dependencies(ping 
  rclcpp
  builtin_interfaces
  )

add_executable(pong src/pong.cpp)
ament_target_dependencies(pong 
  rclcpp
  builtin_interfaces
)

target_compile_features(ping PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(pong PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS 
  ping
  pong
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
