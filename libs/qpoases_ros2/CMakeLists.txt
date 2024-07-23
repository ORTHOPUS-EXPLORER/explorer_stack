cmake_minimum_required(VERSION 3.5)
project(qpoases_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include
  include/qpOASES
)

set(HEADER_FILES
  include/qpOASES/Bounds.hpp
  include/qpOASES/Bounds.ipp
  include/qpOASES/Constants.hpp
  include/qpOASES/ConstraintProduct.hpp
  include/qpOASES/Constraints.hpp
  include/qpOASES/Constraints.ipp
  include/qpOASES/Flipper.hpp
  include/qpOASES/Indexlist.hpp
  include/qpOASES/Indexlist.ipp
  include/qpOASES/LapackBlasReplacement.hpp
  include/qpOASES/Matrices.hpp
  include/qpOASES/MessageHandling.hpp
  include/qpOASES/MessageHandling.ipp
  include/qpOASES/Options.hpp
  include/qpOASES/qpOASES.hpp
  include/qpOASES/QProblem.hpp
  include/qpOASES/QProblem.ipp
  include/qpOASES/QProblemB.hpp
  include/qpOASES/QProblemB.ipp
  include/qpOASES/SparseSolver.hpp
  include/qpOASES/SQProblem.hpp
  include/qpOASES/SQProblem.ipp
  include/qpOASES/SQProblemSchur.hpp
  include/qpOASES/SQProblemSchur.ipp
  include/qpOASES/SubjectTo.hpp
  include/qpOASES/SubjectTo.ipp
  include/qpOASES/Types.hpp
  include/qpOASES/UnitTesting.hpp
  include/qpOASES/Utils.hpp
  include/qpOASES/Utils.ipp
  include/qpOASES/extras/SolutionAnalysis.hpp
  include/qpOASES/extras/SolutionAnalysis.ipp
  include/qpOASES/extras/OQPinterface.hpp
)
set(CPP_FILES
  src/BLASReplacement.cpp
  src/Bounds.cpp
  src/Constraints.cpp
  src/Flipper.cpp
  src/Indexlist.cpp
  src/LAPACKReplacement.cpp
  src/Matrices.cpp
  src/MessageHandling.cpp
  src/Options.cpp
  src/OQPinterface.cpp
  src/QProblem.cpp
  src/QProblemB.cpp
  src/SparseSolver.cpp
  src/SQProblem.cpp
  src/SQProblemSchur.cpp
  src/SubjectTo.cpp
  src/Utils.cpp
  src/SolutionAnalysis.cpp
)
add_library(qpoases_ros2 ${CPP_FILES})

ament_export_targets(qpoases_ros2 HAS_LIBRARY_TARGET)
ament_export_libraries(qpoases_ros2)

install(
  DIRECTORY include/qpOASES
  DESTINATION include
)

install(
  TARGETS qpoases_ros2
  EXPORT qpoases_ros2
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
