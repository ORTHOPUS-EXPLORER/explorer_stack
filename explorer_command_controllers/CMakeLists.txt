cmake_minimum_required(VERSION 3.16)
project(explorer_command_controllers LANGUAGES CXX)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  std_srvs
  orthopus_vesc
  orthopus_vesc_interfaces
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
  explorer_custom_controller_parameters
  src/explorer_custom_controller_parameters.yaml
)
generate_parameter_library(
  explorer_position_saturation_controller_parameters
  src/explorer_position_saturation_controller_parameters.yaml
)

# ROS independent logic, unit tested without ROS
add_library(explorer_position_saturation SHARED
  src/position_saturation.cpp
)
target_compile_features(explorer_position_saturation PUBLIC cxx_std_17)
target_include_directories(explorer_position_saturation
PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/explorer_command_controllers>
)

add_library(explorer_command_controllers SHARED
  src/explorer_custom_controller.cpp
  src/explorer_position_saturation_controller.cpp
)

target_compile_features(explorer_command_controllers 
PUBLIC 
  cxx_std_17
)

target_include_directories(explorer_command_controllers 
PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/explorer_command_controllers>
)

target_link_libraries(explorer_command_controllers 
PUBLIC
  explorer_custom_controller_parameters
  explorer_position_saturation_controller_parameters
  explorer_position_saturation
)

ament_target_dependencies(explorer_command_controllers 
PUBLIC 
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(explorer_command_controllers 
  PRIVATE 
    "EXPLORER_COMMAND_CONTROLLERS_BUILDING_DLL"
)

pluginlib_export_plugin_description_file(controller_interface 
  explorer_command_plugin.xml
)


install(
  DIRECTORY include/
  DESTINATION include/explorer_command_controllers
)
install(
  TARGETS
    explorer_command_controllers
    explorer_custom_controller_parameters
    explorer_position_saturation_controller_parameters
    explorer_position_saturation
  EXPORT export_explorer_command_controllers
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(lifecycle_msgs REQUIRED)

  # Unit tests: ROS independent logic
  ament_add_ros_isolated_gtest(test_position_saturation
    test/test_position_saturation.cpp
  )
  target_link_libraries(test_position_saturation explorer_position_saturation)

  # Component tests: controller ROS interfaces (interfaces, parameters, topic, lifecycle)
  ament_add_ros_isolated_gtest(test_explorer_position_saturation_controller
    test/test_explorer_position_saturation_controller.cpp
  )
  target_link_libraries(test_explorer_position_saturation_controller explorer_command_controllers)
  ament_target_dependencies(test_explorer_position_saturation_controller lifecycle_msgs)
endif()

ament_export_targets(export_explorer_command_controllers 
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
