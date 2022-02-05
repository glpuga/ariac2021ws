cmake_minimum_required(VERSION 3.5)
project(tijchallenger)

add_compile_options(-std=c++17)

add_compile_options(-Wall -Werror)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  control_msgs
  custom_gantry_moveit_config
  custom_kitting_moveit_config
  nist_gear
  roscpp
  rostest
  tf
  tijros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    control_msgs
    custom_gantry_moveit_config
    custom_kitting_moveit_config
    nist_gear
    roscpp
    tf
    tijros
)

# ##############################################################################
# Build ##
# ##############################################################################

add_executable(${PROJECT_NAME}_node
  src/SceneConfigReader.cpp
  src/TIJChallenger.cpp
  src/${PROJECT_NAME}_node.cpp
)
add_dependencies(
  ${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

add_executable(
  infinite_tolerance_node
  src/infinite_tolerance_node.cpp
)
add_dependencies(
  infinite_tolerance_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  infinite_tolerance_node
  ${catkin_LIBRARIES}
)

# ##############################################################################
# Install ##
# ##############################################################################

install(
  TARGETS infinite_tolerance_node ${PROJECT_NAME}_node 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.hpp")
