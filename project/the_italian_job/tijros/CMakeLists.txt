cmake_minimum_required(VERSION 3.5)
project(tijros)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(
  catkin REQUIRED
  COMPONENTS
    geometry_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_visual_tools
    nist_gear
    nist_gear
    roscpp
    roslint
    rostest
    std_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_ros
    tijcore
)

# ##############################################################################
# catkin specific configuration ##
# ##############################################################################

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    geometry_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_visual_tools
    nist_gear
    roscpp
    std_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_ros
    tijcore
)

# ##############################################################################
# Build ##
# ##############################################################################

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_compile_options(-Wall -Werror)

add_library(${PROJECT_NAME}
  src/LogicalCameraModelPerception.cpp
  src/PickAndPlaceAssemblyRobot.cpp
  src/PickAndPlaceKittingRobot.cpp
  src/PickAndPlaceRobotCommonImpl.cpp
  src/QualitySensorModelPerception.cpp
  src/ROSFrameTransformer.cpp
  src/ROSProcessManagement.cpp
  src/ROSRobotActuators.cpp
  src/utils/utils.cpp
)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# ##############################################################################
# Install ##
# ##############################################################################

# Mark libraries for installation See
# http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.hpp")

# ##############################################################################
# Testing ##
# ##############################################################################

if(CATKIN_ENABLE_TESTING)
  # catkin_add_gtest(${PROJECT_NAME}-test test/test_${PROJECT_NAME}.cpp)
  # if(TARGET ${PROJECT_NAME}-test)
  #   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
  # endif()

  # catkin_add_nosetests(test)

  set(ROSLINT_CPP_OPTS --extensions=h,hpp,cpp)
  roslint_cpp()
  roslint_custom(catkin_lint --strict -W2 ${PROJECT_SOURCE_DIR})
  roslint_add_test()

endif()
