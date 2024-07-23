# Subfolders
add_subdirectory(robot_manager/)
add_subdirectory(input_device/)
add_subdirectory(reference_generator/)

# find dependencies
set(CONTROLLER_DEPENDENCIES
  pluginlib
  rcpputils
  controller_interface
  hardware_interface
  realtime_tools
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${CONTROLLER_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

## ros2_control_explorer
add_library(ros2_control_explorer
SHARED
  hardware/explorer_hardware.cpp
  controller/explorer_controller.cpp
)

target_compile_features(ros2_control_explorer
PUBLIC
  cxx_std_17
)

target_include_directories(ros2_control_explorer
PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/hardware/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/controller/include>
  $<INSTALL_INTERFACE:include/ros2_control_explorer>
)

ament_target_dependencies(ros2_control_explorer
PUBLIC
  ${CONTROLLER_DEPENDENCIES}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ros2_control_explorer
PRIVATE
  "ROS2_CONTROL_EXPLORER_BUILDING_DLL"
)

ament_export_libraries(ros2_control_explorer)

install(TARGETS ros2_control_explorer
  EXPORT export_ros2_control_explorer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_ros2_control_explorer
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(${CONTROLLER_DEPENDENCIES}
)
