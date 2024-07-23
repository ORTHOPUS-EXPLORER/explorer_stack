cmake_minimum_required(VERSION 3.16)
project(ros2_control_explorer LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)

add_subdirectory(src/)

# INSTALL
install(DIRECTORY description
                  bringup/launch
                  bringup/config
  DESTINATION share/ros2_control_explorer
)

## EXPORTS
find_package(pluginlib REQUIRED)
# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface
  ros2_control_explorer.xml
)
# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface
  ros2_control_explorer.xml
)

ament_package()
