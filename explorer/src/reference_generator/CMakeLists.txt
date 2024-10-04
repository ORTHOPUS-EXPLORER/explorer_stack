find_package(ament_cmake REQUIRED)

# find dependencies
set(REF_GEN_DEPENDENCIES
  kdl_parser
  rclcpp
  std_msgs
  trajectory_msgs
  sensor_msgs
  std_srvs
  geometry_msgs
  # required to build with explorer_robot_manager
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

foreach(Dependency IN ITEMS ${REF_GEN_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

# spacenav_trajectory
add_executable(spacenav_trajectory
  spacenav_trajectory.cpp
)

target_include_directories(spacenav_trajectory
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include/ros2_control_explorer>
)

ament_target_dependencies(spacenav_trajectory
PUBLIC
  ${REF_GEN_DEPENDENCIES}
)

# spacenav_trajectory_qp
add_executable(spacenav_trajectory_qp
  spacenav_trajectory_qp.cpp
)

target_include_directories(spacenav_trajectory_qp
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include/ros2_control_explorer>
)

ament_target_dependencies(spacenav_trajectory_qp
PUBLIC
  ${REF_GEN_DEPENDENCIES}
)

target_link_libraries(spacenav_trajectory_qp
PRIVATE
  explorer_robot_manager
)

# spacenav_trajectory_qp_pos_only
add_executable(spacenav_trajectory_qp_pos_only
  spacenav_trajectory_qp_pos_only.cpp
)

target_include_directories(spacenav_trajectory_qp_pos_only
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include/ros2_control_explorer>
)

ament_target_dependencies(spacenav_trajectory_qp_pos_only
PUBLIC
  ${REF_GEN_DEPENDENCIES}
)

target_link_libraries(spacenav_trajectory_qp_pos_only
PRIVATE
  explorer_robot_manager
)

# input_integrator
add_executable(input_integrator
  input_integrator.cpp
)

target_include_directories(input_integrator
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include/ros2_control_explorer>
)

ament_target_dependencies(input_integrator
PUBLIC
  ${REF_GEN_DEPENDENCIES}
)

target_link_libraries(input_integrator
PRIVATE
  explorer_robot_manager
)

# qp_solving
add_executable(qp_solving
  qp_solving.cpp
)

target_include_directories(qp_solving
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include/ros2_control_explorer>
)

ament_target_dependencies(qp_solving
PUBLIC
  ${REF_GEN_DEPENDENCIES}
)

target_link_libraries(qp_solving
PRIVATE
  explorer_robot_manager
)

#  Install !
install(TARGETS spacenav_trajectory
                spacenav_trajectory_qp
                spacenav_trajectory_qp_pos_only
                input_integrator
                qp_solving
        DESTINATION lib/ros2_control_explorer
)

ament_export_dependencies(${REF_GEN_DEPENDENCIES}
)


