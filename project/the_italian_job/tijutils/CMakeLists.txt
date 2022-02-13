cmake_minimum_required(VERSION 3.0.2)
project(tijutils)

find_package(catkin REQUIRED COMPONENTS
  gantry_moveit_config
  kitting_moveit_config
  roslint
)

catkin_package(
  CATKIN_DEPENDS
    gantry_moveit_config
    kitting_moveit_config
    roslint
)

#############
## Install ##
#############

install(
  FILES
    launch/rviz_control_gantry.launch
    launch/rviz_control_kitting.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

  set(ROSLINT_CPP_OPTS --extensions=h,hpp,cpp)
  roslint_cpp()
  roslint_custom(catkin_lint --strict -W2 ${PROJECT_SOURCE_DIR})
  roslint_add_test()

endif()