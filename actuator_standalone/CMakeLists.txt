cmake_minimum_required(VERSION 3.5.0)
project(ros2_control_actuator)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(joint_controller src/joint_controller.cpp)
ament_target_dependencies(joint_controller
  rclcpp
  sensor_msgs
  std_msgs
)

install(
  DIRECTORY description
  DESTINATION share/ros2_control_actuator
)

install(
  DIRECTORY src
  DESTINATION share/ros2_control_actuator
)

install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/ros2_control_actuator
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

## Install
install(
  TARGETS
  joint_controller
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()