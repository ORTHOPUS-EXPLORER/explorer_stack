cmake_minimum_required(VERSION 3.16)
project(explorer_gazebo LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)

# INSTALL
install(DIRECTORY worlds
  DESTINATION share/explorer_gazebo
)


ament_package()
