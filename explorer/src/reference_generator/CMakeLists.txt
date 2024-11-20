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
  custom_interfaces
)

foreach(Dependency IN ITEMS ${REF_GEN_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

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

# output_integrator
add_executable(output_integrator
  output_integrator.cpp
)

target_include_directories(output_integrator
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include/ros2_control_explorer>
)

ament_target_dependencies(output_integrator
PUBLIC
  ${REF_GEN_DEPENDENCIES}
)

target_link_libraries(output_integrator
PRIVATE
  explorer_robot_manager
)

# joint_output_integrator
add_executable(joint_output_integrator
  joint_output_integrator.cpp
)


target_include_directories(joint_output_integrator
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include/ros2_control_explorer>
)

ament_target_dependencies(joint_output_integrator
PUBLIC
  ${REF_GEN_DEPENDENCIES}
)

target_link_libraries(joint_output_integrator
PRIVATE
  explorer_robot_manager
)

# test_output_integrator
add_executable(test_output_integrator
  test_output_integrator.cpp
)

target_include_directories(test_output_integrator
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include/ros2_control_explorer>
)

ament_target_dependencies(test_output_integrator
PUBLIC
  ${REF_GEN_DEPENDENCIES}
)

target_link_libraries(test_output_integrator
PRIVATE
  explorer_robot_manager
)

#  Install !
install(TARGETS input_integrator
                qp_solving
                output_integrator
                joint_output_integrator
                test_output_integrator
        DESTINATION lib/ros2_control_explorer
)

ament_export_dependencies(${REF_GEN_DEPENDENCIES}
)


