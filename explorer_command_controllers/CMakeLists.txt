cmake_minimum_required(VERSION 3.16)
project(explorer_command_controllers LANGUAGES CXX)

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
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
  explorer_multi_intf_controller_parameters
  src/explorer_multi_intf_controller_parameters.yaml
)

add_library(explorer_command_controllers SHARED
  src/explorer_multi_intf_controller.cpp
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
  explorer_multi_intf_controller_parameters
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
    explorer_multi_intf_controller_parameters
  EXPORT export_explorer_command_controllers
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_explorer_command_controllers 
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
