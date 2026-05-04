cmake_minimum_required(VERSION 3.22)  # Add this line to specify the minimum CMake version
project(explorer_input_devices)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# find dependencies
set(SPACENAV_DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
  geometry_msgs
  joy
)
foreach(Dependency IN ITEMS ${SPACENAV_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

# INSTALL
install(DIRECTORY config
  DESTINATION share/${CMAKE_PROJECT_NAME}
)

add_executable(spacenav
  src/device.cpp
  src/device_spacenav.cpp
)

target_include_directories(spacenav
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include/explorer_input_devices>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(spacenav
  ${SPACENAV_DEPENDENCIES}
)

add_executable(xbox_gamepad_joint
  src/xbox_gamepad_joint.cpp
)

target_include_directories(xbox_gamepad_joint
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include/explorer_input_devices>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(xbox_gamepad_joint
  ${SPACENAV_DEPENDENCIES}
)

install(TARGETS spacenav
                xbox_gamepad_joint
        DESTINATION lib/explorer_input_devices
)

ament_package()