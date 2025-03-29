cmake_minimum_required(VERSION 3.22)
project(orthopus_vesc_interface)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(orthopus_vesc REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(${PROJECT_NAME}
  src/orthopus_vesc_interface.cpp
)

add_library(orthopus_ros::${PROJECT_NAME}
  ALIAS ${PROJECT_NAME}
)

target_compile_features(${PROJECT_NAME}
  PUBLIC
    c_std_99
    cxx_std_17
)
target_include_directories(${PROJECT_NAME}
PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  orthopus_vesc
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME}
PRIVATE
  "ORTHOPUS_ROS_BUILDING_LIBRARY"
)


install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(
  hardware_interface
    orthopus_vesc_interface.xml
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
