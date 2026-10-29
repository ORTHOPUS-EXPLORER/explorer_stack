cmake_minimum_required(VERSION 3.8)
project(explorer_user_interfaces_cpp)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(backward_ros REQUIRED)

set(INTERFACES_DEPENDENCIES
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  explorer_msgs
  explorer_leds
  explorer_joint_utils
  joy
  controller_manager
  controller_manager_msgs
)

foreach(Dependency IN ITEMS ${INTERFACES_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

# LED status config loading and command selection, ROS independent
add_library(led_status_config STATIC
  src/led_status_config.cpp
)
target_include_directories(led_status_config
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
)
ament_target_dependencies(led_status_config
  explorer_msgs
  rclcpp
  trajectory_msgs
  yaml_cpp_vendor
)
target_link_libraries(led_status_config yaml-cpp)

add_executable(command_node
  src/command_node.cpp
  src/trajectory_manager.cpp
  src/controller_manager_wrapper.cpp
  )

target_include_directories(command_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
    $<INSTALL_INTERFACE:include/explorer_user_interfaces_cpp>
)

ament_target_dependencies(command_node
  ${INTERFACES_DEPENDENCIES}
  yaml_cpp_vendor
)

target_link_libraries(command_node 
  led_status_config
  yaml-cpp
)

install(TARGETS command_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  bringup/launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY 
  bringup/config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ## Helper function
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()

  ## UNIT TESTS
  ament_add_ros_isolated_gtest(test_led_status_config test/test_led_status_config.cpp)
  target_link_libraries(test_led_status_config led_status_config)
  target_compile_definitions(test_led_status_config PRIVATE
    SHIPPED_LED_STATUS_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/bringup/config/led_status_mode_1.yaml")

  ## LAUNCH & E2E TESTS
  add_ros_isolated_launch_test(test/launch/test_mode_0.py)
  add_ros_isolated_launch_test(test/launch/test_mode_0_impedance.py)
  
endif()

ament_package()
