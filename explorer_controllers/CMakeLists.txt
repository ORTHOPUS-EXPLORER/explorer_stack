cmake_minimum_required(VERSION 3.22)  # Add this line to specify the minimum CMake version
project(explorer_controllers)

find_package(ament_cmake REQUIRED)

# find dependencies
set(CONTROLLERS_DEPENDENCIES
  pluginlib
  rcpputils
  realtime_tools
  control_msgs
  kdl_parser
  rclcpp
  std_msgs
  trajectory_msgs
  sensor_msgs
  std_srvs
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  explorer_msgs
  joy
  controller_interface
  realtime_tools
  Eigen3
  qpoases_ros2
)
foreach(Dependency IN ITEMS ${CONTROLLERS_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()


add_executable(input_integrator
  src/common/input_integrator.cpp
)

target_include_directories(input_integrator
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include/explorer_controllers>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(input_integrator
  ${CONTROLLERS_DEPENDENCIES}
)

target_link_libraries(input_integrator
  controllers
)

add_executable(output_integrator
  src/common/output_integrator.cpp
)

target_include_directories(output_integrator
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include/explorer_controllers>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(output_integrator
  ${CONTROLLERS_DEPENDENCIES}
)

add_executable(joint_output_integrator
  src/qp_cartesian/joint_output_integrator.cpp
)

target_link_libraries(output_integrator
  controllers
)

target_include_directories(joint_output_integrator
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include/explorer_controllers>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(joint_output_integrator
  ${CONTROLLERS_DEPENDENCIES}
)

target_link_libraries(joint_output_integrator
  controllers
)

add_executable(qp_solving
  src/qp_cartesian/qp_solving.cpp
)

target_include_directories(qp_solving
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include/explorer_controllers>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(qp_solving
  ${CONTROLLERS_DEPENDENCIES}
)

target_link_libraries(qp_solving
  controllers
)

install(TARGETS input_integrator
                output_integrator
                joint_output_integrator
                qp_solving
        DESTINATION lib/explorer_controllers
)

add_library(controllers STATIC
  src/qp_cartesian/forward_kinematic.cpp
  src/qp_cartesian/inverse_kinematic.cpp
  src/common/velocity_integrator.cpp
  src/qp_cartesian/types/space_base.cpp
)

target_include_directories(controllers
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include/explorer_controllers>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(controllers
PUBLIC
  ${CONTROLLERS_DEPENDENCIES}
)

ament_package()