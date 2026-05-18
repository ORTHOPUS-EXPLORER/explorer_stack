cmake_minimum_required(VERSION 3.5)
project(qpoases_ros2)

find_package(ament_cmake REQUIRED)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install static library
install(FILES libqpoases_ros2.a
  DESTINATION lib
)

# Install CMake 'find' file
install(FILES
  cmake/qpoases_ros2Config.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)