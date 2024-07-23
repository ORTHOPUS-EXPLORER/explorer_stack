set(ROBOT_MGR_DEPENDENCIES
  rclcpp
  pluginlib
  rcpputils
  controller_interface
  hardware_interface
  realtime_tools
  Eigen3
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  qpoases_ros2
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${ROBOT_MGR_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(explorer_robot_manager STATIC
  forward_kinematic.cpp
  inverse_kinematic.cpp
  velocity_integrator.cpp
  types/space_base.cpp
)

target_include_directories(explorer_robot_manager
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include/ros2_control_explorer>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(explorer_robot_manager
PUBLIC
  ${ROBOT_MGR_DEPENDENCIES}
)

#install(TARGETS explorer_robot_manager
#  RUNTIME DESTINATION lib/ros2_control_explorer
#)