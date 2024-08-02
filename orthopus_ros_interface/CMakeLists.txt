cmake_minimum_required(VERSION 3.8)
project(orthopus_ros_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)


add_library(orthopus_ros_interface
  src/hw_interface.cpp
)

add_library(orthopus_ros_interface::orthopus_ros_interface
  ALIAS orthopus_ros_interface
)
target_compile_features(orthopus_ros_interface
  PUBLIC
    c_std_99
    cxx_std_17
)
target_include_directories(orthopus_ros_interface
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(orthopus_ros_interface
PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(orthopus_ros_interface
PRIVATE
  "ORTHOPUS_ROS_BUILDING_LIBRARY"
)


install(
  TARGETS orthopus_ros_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  orthopus_ros_interface
)

# Export hardware plugins
pluginlib_export_plugin_description_file(
  hardware_interface
    orthopus_ros_interface.xml
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
