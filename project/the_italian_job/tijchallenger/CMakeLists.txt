cmake_minimum_required(VERSION 2.8.3)
project(tijchallenger)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

add_compile_options(-Wall -Werror)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  control_msgs
  tijcore
  tijros
  roscpp
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(tijchallenger_node
  src/SceneConfigReader.cpp
  src/TIJChallenger.cpp  
  src/tijchallenger_node.cpp
)

add_dependencies(tijchallenger_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(tijchallenger_node
  ${catkin_LIBRARIES}
)

add_executable(infinite_tolerance_node
  src/infinite_tolerance_node.cpp
)

add_dependencies(infinite_tolerance_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(infinite_tolerance_node
  ${catkin_LIBRARIES}
)


