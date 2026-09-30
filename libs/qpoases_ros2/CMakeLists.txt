cmake_minimum_required(VERSION 3.5)
project(qpoases_ros2)

find_package(ament_cmake REQUIRED)

# Declare imported target for the prebuilt static lib
add_library(${PROJECT_NAME} INTERFACE)

# Note the SYSTEM: prevent from generating build warnings
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} INTERFACE
  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/lib/lib${PROJECT_NAME}.a>"
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install static library
install(FILES lib${PROJECT_NAME}.a
  DESTINATION lib
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()