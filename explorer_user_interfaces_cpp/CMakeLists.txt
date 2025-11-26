cmake_minimum_required(VERSION 3.8)
project(explorer_user_interfaces_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)


set(INTERFACES_DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  explorer_msgs
  joy
)

foreach(Dependency IN ITEMS ${INTERFACES_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()


add_executable(command_node 
  src/command_node.cpp
  src/button_handler.cpp
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

ament_package()
