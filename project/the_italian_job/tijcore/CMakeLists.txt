cmake_minimum_required(VERSION 3.5)
project(tijcore)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
add_compile_options(-Wall -Werror)

find_package(catkin REQUIRED COMPONENTS
  behavior_tree_extras
  behaviortree_cpp_v3
  roscpp
  roslint
  rostest
  tijlogger
  tijmath
  tijutils
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    behavior_tree_extras
    behaviortree_cpp_v3
    roscpp
    tijlogger
    tijutils
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
  src/AgvId.cpp
  src/AgvModelContainer.cpp
  src/AssemblyStationModelContainer.cpp
  src/BTRobotTaskFactory.cpp
  src/BehaviorTreeTransitionLogger.cpp
  src/BinId.cpp
  src/BinModelContainer.cpp
  src/ConveyorBeltModelContainer.cpp
  src/CuboidVolume.cpp
  src/GripperTypeId.cpp
  src/ManagedLocus.cpp
  src/ModelContainerInterface.cpp
  src/ModelPerceptionMixer.cpp
  src/ModelPerceptionSpatialFilter.cpp
  src/MovableTrayId.cpp
  src/OrderId.cpp
  src/OrderProcessingStrategy.cpp
  src/PartColorId.cpp
  src/PartId.cpp
  src/PartTypeId.cpp
  src/PickAndPlaceTask.cpp
  src/RemoveBrokenPartTask.cpp
  src/ResourceHandle.cpp
  src/ResourceManager.cpp
  src/RobotTaskFactory.cpp
  src/RobotTaskGroupRunner.cpp
  src/RobotTaskOutcome.cpp
  src/SpatialMutualExclusionManager.cpp
  src/StaticFrameTransformer.cpp
  src/StationId.cpp
  src/SubmitAssemblyShipmentTask.cpp
  src/SubmitKittingShipmentTask.cpp
  src/SurfaceManager.cpp
  src/TableModelContainer.cpp
  src/TaskDispatcher.cpp
  src/TaskDriver.cpp
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
  catkin_add_gmock(${fast_test_target_name} tests/runner.cpp ${fast_test_files_list} WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/tests)
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