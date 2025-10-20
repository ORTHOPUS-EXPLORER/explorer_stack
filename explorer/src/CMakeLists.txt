# Subfolders
add_subdirectory(robot_manager/)
add_subdirectory(reference_generator/)

# find dependencies
set(CONTROLLER_DEPENDENCIES
  pluginlib
  rcpputils
  controller_interface
  hardware_interface
  realtime_tools
  control_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${CONTROLLER_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()
