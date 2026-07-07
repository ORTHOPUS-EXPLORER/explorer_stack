cmake_minimum_required(VERSION 3.16)
project(explorer_description LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)

# INSTALL
install(DIRECTORY gazebo
                  launch
                  meshes
                  ros2_control
                  rviz
                  urdf
  DESTINATION share/${CMAKE_PROJECT_NAME}
)


ament_package()
