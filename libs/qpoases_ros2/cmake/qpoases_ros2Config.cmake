set(qpoases_ros2_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
set(qpoases_ros2_LIBRARIES "${CMAKE_INSTALL_PREFIX}/lib/libqpoases_ros2.a")

add_library(qpoases_ros2 STATIC IMPORTED)
set_target_properties(qpoases_ros2 PROPERTIES
  IMPORTED_LOCATION "${CMAKE_INSTALL_PREFIX}/lib/libqpoases_ros2.a"
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/include"
)