cmake_minimum_required(VERSION 3.8)
project(explorer_leds)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

set(LEDS_DEPENDENCIES
  rclcpp
  explorer_msgs
  std_msgs
)

foreach(Dependency IN ITEMS ${LEDS_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

# Animation and controller logic, LED encoding and strip config, ROS independent
add_library(led_core STATIC
  src/led_animation.cpp
  src/led_controller.cpp
  src/led_strip_config.cpp
  src/sk6812_encoder.cpp
)
target_include_directories(led_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
    $<INSTALL_INTERFACE:include/explorer_leds>
)
set_target_properties(led_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(led_core yaml-cpp)

# ROS wrapper and hardware driver
add_library(led_node_lib STATIC
  src/led_node.cpp
  src/radxa_led_driver.cpp
)
target_link_libraries(led_node_lib led_core)
ament_target_dependencies(led_node_lib ${LEDS_DEPENDENCIES} ament_index_cpp)

add_executable(led_node
  src/main.cpp
)
target_link_libraries(led_node led_node_lib)

install(TARGETS led_node
  DESTINATION lib/${PROJECT_NAME}
)

# Headers, for publishers to use led_command_qos()
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  bringup/launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)

  ## UNIT TESTS
  ament_add_ros_isolated_gtest(test_led_animation test/test_led_animation.cpp)
  target_link_libraries(test_led_animation led_core)

  ament_add_ros_isolated_gtest(test_sk6812_encoder test/test_sk6812_encoder.cpp)
  target_link_libraries(test_sk6812_encoder led_core)

  ament_add_ros_isolated_gtest(test_led_strip_config test/test_led_strip_config.cpp)
  target_link_libraries(test_led_strip_config led_core)

  ament_add_ros_isolated_gtest(test_led_controller test/test_led_controller.cpp)
  target_link_libraries(test_led_controller led_core)

  ## ROS UNIT TESTS
  ament_add_ros_isolated_gtest(test_led_node test/test_led_node.cpp)
  target_link_libraries(test_led_node led_node_lib)
endif()

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)

ament_package()
