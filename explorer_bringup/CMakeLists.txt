cmake_minimum_required(VERSION 3.16)
project(explorer_bringup LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# INSTALL
install(DIRECTORY 
launch
config
DESTINATION share/${CMAKE_PROJECT_NAME}
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
  add_ros_isolated_launch_test(test/launch/test_custom_controller_joint_control.py)
  
endif()

ament_python_install_package(${PROJECT_NAME})
ament_package()
