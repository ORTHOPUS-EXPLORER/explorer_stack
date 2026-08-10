cmake_minimum_required(VERSION 3.22)  # Add this line to specify the minimum CMake version
project(explorer_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)

find_package(pinocchio QUIET)
if(NOT pinocchio_FOUND)
  if(DEFINED ENV{PINOCCHIO_DIR})
    list(APPEND CMAKE_PREFIX_PATH "$ENV{PINOCCHIO_DIR}")
    find_package(pinocchio QUIET)
  endif()
  if(NOT pinocchio_FOUND)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
      pkg_check_modules(PINOCCHIO QUIET pinocchio)
      if(PINOCCHIO_FOUND)
        set(pinocchio_FOUND TRUE)
        set(pinocchio_INCLUDE_DIRS ${PINOCCHIO_INCLUDE_DIRS})
        set(pinocchio_LIBRARIES ${PINOCCHIO_LIBRARIES})
      endif()
    endif()
  endif()
  if(NOT pinocchio_FOUND)
    message(FATAL_ERROR "Pinocchio could not be found. Install Pinocchio (for example via apt or from source) and set PINOCCHIO_DIR/pinocchio_DIR or PKG_CONFIG_PATH accordingly.")
  endif()
endif()

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
  pinocchio
  orthopus_vesc_interfaces
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
  ${pinocchio_INCLUDE_DIRS}
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
  ${pinocchio_INCLUDE_DIRS}
)

ament_target_dependencies(output_integrator
  ${CONTROLLERS_DEPENDENCIES}
)

add_executable(gravity_compensation_node
  src/common/gravity_compensation_node.cpp
)

target_include_directories(gravity_compensation_node
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include/explorer_controllers>
  ${EIGEN3_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
)

ament_target_dependencies(gravity_compensation_node
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
  ${pinocchio_INCLUDE_DIRS}
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
  ${pinocchio_INCLUDE_DIRS}
)

ament_target_dependencies(qp_solving
  ${CONTROLLERS_DEPENDENCIES}
)

target_link_libraries(qp_solving
  controllers
)


install(TARGETS input_integrator
                output_integrator
                gravity_compensation_node
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
  ${pinocchio_INCLUDE_DIRS}
)

ament_target_dependencies(controllers
PUBLIC
  ${CONTROLLERS_DEPENDENCIES}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()