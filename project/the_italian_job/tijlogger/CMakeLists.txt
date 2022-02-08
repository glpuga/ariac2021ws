cmake_minimum_required(VERSION 3.5)
project(tijlogger)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
add_compile_options(-Wall -Werror)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  rostest
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
)

# ##############################################################################
# Build ##
# ##############################################################################

include_directories(
  include
)

# ##############################################################################
# Install ##
# ##############################################################################

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

# ##############################################################################
# Testing ##
# ##############################################################################

if(CATKIN_ENABLE_TESTING)

  file(GLOB fast_test_files_list tests/fast_tests_*.cpp)
  set(fast_test_target_name ${PROJECT_NAME}-fast-tests)
  catkin_add_gmock(${fast_test_target_name} tests/runner.cpp ${fast_test_files_list})
  target_link_libraries(${fast_test_target_name} ${catkin_LIBRARIES})

  # This is to exclude spdlog from getting checked
  file(GLOB_RECURSE CPPFILES
    include/${PROJECT_NAME}/*.h
    include/${PROJECT_NAME}/*.hpp
    src/*.cpp
    tests/*.hpp
    tests/*.cpp)
  list(FILTER CPPFILES EXCLUDE REGEX ".*/spdlog/.*" )

  set(ROSLINT_CPP_OPTS --extensions=h,hpp,cpp)
  roslint_cpp(${CPPFILES})
  
  roslint_custom(catkin_lint --strict -W2 ${PROJECT_SOURCE_DIR})
  roslint_add_test()

endif()