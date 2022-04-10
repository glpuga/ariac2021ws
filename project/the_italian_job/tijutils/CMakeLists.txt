cmake_minimum_required(VERSION 3.0.2)
project(tijutils)

find_package(catkin REQUIRED COMPONENTS
  gantry_moveit_config
  kitting_moveit_config
  roslint
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    gantry_moveit_config
    kitting_moveit_config
    roslint
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
  src/Timer.cpp
  src/UniqueId.cpp
  src/string_utils.cpp
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
)

install(
  FILES
    launch/rviz_control_gantry.launch
    launch/rviz_control_kitting.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# ##############################################################################
# Testing ##
# ##############################################################################

if(CATKIN_ENABLE_TESTING)
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
