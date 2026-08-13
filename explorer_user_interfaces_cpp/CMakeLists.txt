cmake_minimum_required(VERSION 3.8)
project(explorer_user_interfaces_cpp)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(backward_ros REQUIRED)

set(INTERFACES_DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  trajectory_msgs
  explorer_msgs
  joy
  controller_manager
  controller_manager_msgs
)

foreach(Dependency IN ITEMS ${INTERFACES_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()


add_executable(command_node 
  src/command_node.cpp
  src/button_handler.cpp
  src/trajectory_manager.cpp
  src/controller_manager_wrapper.cpp
  )

target_include_directories(command_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
    $<INSTALL_INTERFACE:include/explorer_user_interfaces_cpp>
)

ament_target_dependencies(command_node
  ${INTERFACES_DEPENDENCIES}
  yaml_cpp_vendor
)

target_link_libraries(command_node 
  yaml-cpp
)

install(TARGETS command_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  bringup/launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY 
  bringup/config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ## Helper function
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()

  ## LAUNCH & E2E TESTS
  add_ros_isolated_launch_test(test/launch/test_mode_0.py)
  add_ros_isolated_launch_test(test/launch/test_mode_0_impedance.py)
  
endif()

ament_package()
