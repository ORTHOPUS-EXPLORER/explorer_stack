cmake_minimum_required(VERSION 3.22)  # Add this line to specify the minimum CMake version
project(explorer_hardware)

find_package(ament_cmake REQUIRED)

# find dependencies
set(HARDWARE_DEPENDENCIES
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

foreach(Dependency IN ITEMS ${HARDWARE_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()


add_library(hardware STATIC
  src/explorer_hardware.cpp

)

target_include_directories(hardware
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include/explorer_hardware>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(hardware
PUBLIC
  ${HARDWARE_DEPENDENCIES}
)

find_package(pluginlib REQUIRED)
# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface
  explorer_hardware.xml
)

ament_package()