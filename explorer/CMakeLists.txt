cmake_minimum_required(VERSION 3.16)
project(ros2_control_explorer LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)

add_subdirectory(src/)

# INSTALL
install(DIRECTORY description
                  bringup/launch
                  bringup/config
  DESTINATION share/ros2_control_explorer
)

## EXPORTS
ament_export_targets(export_ros2_control_explorer
  HAS_LIBRARY_TARGET
)
ament_export_dependencies(${CONTROLLER_INCLUDE_DEPENDS}
)
ament_package()
