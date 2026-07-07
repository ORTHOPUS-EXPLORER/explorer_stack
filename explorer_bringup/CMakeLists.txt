cmake_minimum_required(VERSION 3.16)
project(explorer_bringup LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)

# INSTALL
install(DIRECTORY 
                  launch
                  config
  DESTINATION share/${CMAKE_PROJECT_NAME}
)


ament_package()
