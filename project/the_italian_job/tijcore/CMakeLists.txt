cmake_minimum_required(VERSION 3.0.2)
project(tijcore)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

# System dependencies are found with CMake's conventions find_package(Boost
# REQUIRED COMPONENTS system)

# Uncomment this if the package has a setup.py. This macro ensures modules and
# global scripts declared therein get installed See
# http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  tijcore
  CATKIN_DEPENDS
  roscpp
  rospy
  DEPENDS Eigen
)

# ##############################################################################
# Build ##
# ##############################################################################

include_directories(include ${catkin_INCLUDE_DIRS}   ${Eigen_INCLUDE_DIRS})

add_compile_options(-Wall -Werror)

# TODO(glpuga) The folder structure of the project should be revised and files redistributed. Currently almost everything is under
# perception just for the lack of better named folders.
add_library(
  ${PROJECT_NAME}
  src/agents/AgvId.cpp
  src/agents/BinId.cpp
  src/agents/PartId.cpp
  src/agents/PartColorId.cpp
  src/agents/PartTypeId.cpp
  src/agents/OrderId.cpp
  src/agents/StationId.cpp
  src/agents/RobotTaskOutcome.cpp
  src/agents/SurfaceManager.cpp
  src/agents/WorkRegionId.cpp
  src/localization/FrameId.cpp
  src/localization/Pose3.cpp
  src/localization/RelativePose3.cpp
  src/math/CuboidVolume.cpp
  src/math/Isometry.cpp
  src/math/Matrix3.cpp
  src/math/Position.cpp
  src/math/Quaternion.cpp
  src/math/Rotation.cpp
  src/math/Vector3.cpp
  src/perception/AgvModelContainer.cpp
  src/perception/AssemblyStationModelContainer.cpp
  src/perception/BinModelContainer.cpp
  src/perception/ManagedLocus.cpp
  src/perception/ModelContainerInterface.cpp
  src/perception/ModelPerceptionMixer.cpp
  src/perception/OrderProcessingStrategy.cpp
  src/perception/ResourceHandle.cpp
  src/perception/ResourceManager.cpp
  src/perception/RobotTaskFactory.cpp
  src/perception/TaskMaster.cpp
  src/perception/TaskDriver.cpp
  src/perception/StaticFrameTransformer.cpp
  src/tasks/PickAndPlaceTask.cpp
  src/tasks/RemoveBrokenPartTask.cpp
  src/tasks/RobotTaskGroupRunner.cpp
  src/tasks/SubmitAssemblyShipmentTask.cpp
  src/tasks/SubmitKittingShipmentTask.cpp
  src/utils/angles.cpp
  src/utils/Timer.cpp
  src/utils/string.cpp)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# ##############################################################################
# Testing ##
# ##############################################################################

file(GLOB fast_test_files_list tests/fast_tests_*.cpp)
set(fast_test_target_name ${PROJECT_NAME}-fast-tests)
catkin_add_gmock(${fast_test_target_name} tests/runner.cpp ${fast_test_files_list})
target_link_libraries(${fast_test_target_name} ${PROJECT_NAME} ${catkin_LIBRARIES})

file(GLOB slow_test_files_list tests/slow_tests_*.cpp)
set(slow_test_target_name ${PROJECT_NAME}-slow-tests)
catkin_add_gmock(${slow_test_target_name} tests/runner.cpp ${slow_test_files_list})
target_link_libraries(${slow_test_target_name} ${PROJECT_NAME} ${catkin_LIBRARIES})

# Add folders to be run by python nosetests

# catkin_add_nosetests(test)
