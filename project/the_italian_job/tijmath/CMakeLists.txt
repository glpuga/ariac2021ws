cmake_minimum_required(VERSION 3.5)
project(tijmath)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
add_compile_options(-Wall -Werror)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  rostest
  tijlogger
)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    tijlogger
  DEPENDS
    EIGEN3
)

# ##############################################################################
# Build ##
# ##############################################################################

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/FrameId.cpp
  src/Isometry.cpp
  src/Matrix3.cpp
  src/Pose3.cpp
  src/Position.cpp
  src/Quaternion.cpp
  src/RelativePose3.cpp
  src/Rotation.cpp
  src/Vector3.cpp
  src/math_utilities.cpp
)

add_dependencies(
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# ##############################################################################
# Install ##
# ##############################################################################

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

# ##############################################################################
# Testing ##
# ##############################################################################

if(CATKIN_ENABLE_TESTING)
  # catkin_add_gtest(${PROJECT_NAME}-test test/test_${PROJECT_NAME}.cpp)
  # if(TARGET ${PROJECT_NAME}-test)
  #   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
  # endif()

  # catkin_add_nosetests(test)

  file(GLOB fast_test_files_list tests/fast_tests_*.cpp)
  set(fast_test_target_name ${PROJECT_NAME}-fast-tests)
  catkin_add_gmock(${fast_test_target_name} tests/runner.cpp ${fast_test_files_list})
  target_link_libraries(${fast_test_target_name} ${PROJECT_NAME} ${catkin_LIBRARIES})

  file(GLOB slow_test_files_list tests/slow_tests_*.cpp)
  set(slow_test_target_name ${PROJECT_NAME}-slow-tests)
  catkin_add_gmock(${slow_test_target_name} tests/runner.cpp ${slow_test_files_list})
  target_link_libraries(${slow_test_target_name} ${PROJECT_NAME} ${catkin_LIBRARIES})

  set(ROSLINT_CPP_OPTS --extensions=h,hpp,cpp)
  roslint_cpp()

  roslint_custom(catkin_lint --strict -W2 ${PROJECT_SOURCE_DIR})
  roslint_add_test()

endif()