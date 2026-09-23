cmake_minimum_required(VERSION 3.22)  # Add this line to specify the minimum CMake version
project(explorer_input_devices)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# find dependencies
find_package(yaml-cpp REQUIRED)

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

# The joystick device additionally publishes the control frame selection.
set(JOYSTICK_DEPENDENCIES
  ${SPACENAV_DEPENDENCIES}
  explorer_msgs
)
find_package(explorer_msgs REQUIRED)

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

add_executable(device_joystick
  src/device.cpp
  src/device_joystick.cpp
  src/button_handler.cpp
  src/mode_config.cpp
)

target_include_directories(device_joystick
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include/explorer_input_devices>
)

ament_target_dependencies(device_joystick
  ${JOYSTICK_DEPENDENCIES}
)

target_link_libraries(device_joystick
  yaml-cpp
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

add_executable(joystick_selector
  src/joystick_selector.cpp
)

target_include_directories(joystick_selector
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include/explorer_input_devices>
)

ament_target_dependencies(joystick_selector
  ${SPACENAV_DEPENDENCIES}
)

install(TARGETS spacenav
                device_joystick
                joystick_selector
                xbox_gamepad_joint
        DESTINATION lib/explorer_input_devices
)

ament_package()