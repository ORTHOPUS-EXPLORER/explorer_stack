# spacenav
find_package(ament_cmake REQUIRED)

# find dependencies
set(SPACENAV_DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
  geometry_msgs
)
foreach(Dependency IN ITEMS ${SPACENAV_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(spacenav
  device.cpp
  device_spacenav.cpp
)

target_include_directories(spacenav
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include/ros2_control_explorer>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(spacenav
  ${SPACENAV_DEPENDENCIES}
)

install(TARGETS spacenav
        DESTINATION lib/ros2_control_explorer
)