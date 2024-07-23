cmake_minimum_required(VERSION 3.16)
project(ros2_control_explorer LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
)
set(REF_GEN_INCLUDE_DEPENDS
  kdl_parser
  rclcpp
  std_msgs
  trajectory_msgs
  sensor_msgs
  std_srvs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  Eigen3
  custom_interfaces
  geometry_msgs
  qpoases_ros2
  trac_ik_lib
)
set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  realtime_tools
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${REF_GEN_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE

add_executable(spacenav_trajectory
  src/reference_generator/spacenav_trajectory.cpp
)

ament_target_dependencies(spacenav_trajectory
PUBLIC
  ${REF_GEN_INCLUDE_DEPENDS}
)

add_executable(spacenav_trajectory_ik
  src/reference_generator/spacenav_trajectory_ik.cpp
)
ament_target_dependencies(spacenav_trajectory_ik
PUBLIC
  ${REF_GEN_INCLUDE_DEPENDS}
)

add_executable(spacenav_trajectory_qp
  src/reference_generator/spacenav_trajectory_qp.cpp
  src/robot_manager/forward_kinematic.cpp
  src/robot_manager/inverse_kinematic.cpp
  src/robot_manager/velocity_integrator.cpp
  src/robot_manager/types/space_base.cpp
)

ament_target_dependencies(spacenav_trajectory_qp
PUBLIC
  ${REF_GEN_INCLUDE_DEPENDS}
)

add_executable(send_trajectory
  src/reference_generator/send_trajectory.cpp
)

ament_target_dependencies(send_trajectory
PUBLIC
  ${REF_GEN_INCLUDE_DEPENDS}
)

add_library(ros2_control_explorer
SHARED
  src/hardware/explorer_hardware.cpp
  src/controller/explorer_controller.cpp
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
  ${HW_IF_INCLUDE_DEPENDS}
  ${CONTROLLER_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ros2_control_explorer
PRIVATE
  "ROS2_CONTROL_EXPLORER_BUILDING_DLL"
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface
  ros2_control_explorer.xml
)
# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface
  ros2_control_explorer.xml
)

# INSTALL

install(DIRECTORY description
                  #reference_generator
                  bringup/launch
                  bringup/config
  DESTINATION share/ros2_control_explorer
)

include_directories(
  src/input_device/include
  src/robot_manager/include
  src/reference_generator/include
  ${EIGEN3_INCLUDE_DIRS}
)

install(TARGETS send_trajectory
  RUNTIME DESTINATION lib/ros2_control_explorer
)

install(TARGETS ros2_control_explorer
  EXPORT export_ros2_control_explorer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

############ Space Nav
add_executable(spacenav
  src/input_device/device.cpp
  src/input_device/device_spacenav.cpp
)
ament_target_dependencies(spacenav
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
  geometry_msgs
)

#Install Cpp executables
install(TARGETS spacenav
        DESTINATION lib/ros2_control_explorer
)

#Install Cpp executables
install(TARGETS spacenav_trajectory
                spacenav_trajectory_ik
                spacenav_trajectory_qp
        DESTINATION lib/ros2_control_explorer
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_ros2_control_explorer
  HAS_LIBRARY_TARGET
)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS}
                          ${REF_GEN_INCLUDE_DEPENDS}
                          ${CONTROLLER_INCLUDE_DEPENDS}
)
ament_package()
