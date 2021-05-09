/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// standard library
#include <chrono>

// tijcore
#include <logger/logger.hpp>
#include <tijcore/tasks/ModelTrayAccessSpaceManager.hpp>
#include <tijcore/tasks/PickAndPlaceTask.hpp>

namespace tijcore {

namespace {

constexpr std::chrono::seconds timeout_{120};
}

PickAndPlaceTask::PickAndPlaceTask(
    const ResourceManagerInterface::SharedPtr &resource_manager,
    ResourceManagerInterface::ManagedLocusHandle &&source,
    ResourceManagerInterface::ManagedLocusHandle &&destination,
    ResourceManagerInterface::PickAndPlaceRobotHandle &&robot)
    : resource_manager_{resource_manager}, source_{std::move(source)},
      destination_{std::move(destination)}, robot_{std::move(robot)} {}

RobotTaskOutcome PickAndPlaceTask::run() {
  auto &robot = *robot_.resource();
  RobotTaskOutcome result{RobotTaskOutcome::TASK_FAILURE};

  ModelTrayAccessSpaceManager model_tray_access_manager(*resource_manager_,
                                                        robot);

  // clear exclusion zones to enable movement to a safe spot regardless of where
  // we are located
  model_tray_access_manager.clearAllExclusionZones();

  const auto source_parent_name = source_.resource()->parentName();
  const auto destination_parent_name = destination_.resource()->parentName();

  // if we don't change exclusion zones, we can skip some time-consuming steps
  const bool do_exclusion_zone_change =
      resource_manager_->getContainerExclusionZoneId(source_parent_name) !=
      resource_manager_->getContainerExclusionZoneId(destination_parent_name);

  if (!robot.getInSafePoseNearTarget(source_.resource()->pose()) ||
      !model_tray_access_manager.releaseAccess()) {
    ERROR("{} failed to get in resting pose", robot.name());
  } else if ((do_exclusion_zone_change &&
              !model_tray_access_manager.getAccessToModel(source_parent_name,
                                                          timeout_)) ||
             (!do_exclusion_zone_change &&
              !model_tray_access_manager.getAccessToModel(
                  source_parent_name, destination_parent_name, timeout_))) {
    ERROR("{} failed to setup access constraints to target", robot.name());
  } else if (!robot.getToGraspingPoseHint(source_.resource()->pose())) {
    ERROR("{} failed to get closer to target", robot.name());
  } else if (!robot.getInLandingSpot(source_.resource()->pose())) {
    ERROR("{} failed to get into the landing pose prior to grasping",
          robot.name());
  } else if (!robot.graspPartFromAbove(source_.resource()->pose())) {
    ERROR("{} failed to grasp the part form the surface", robot.name());
  } else if (!robot.getInLandingSpot(source_.resource()->pose()) ||
             !robot.gripperHasPartAttached()) {
    ERROR("{} failed to get into the landing pose post grasping with the part "
          "grasped",
          robot.name());
  } else if (do_exclusion_zone_change &&
             (!robot.getInSafePoseNearTarget(source_.resource()->pose()) ||
              !model_tray_access_manager.releaseAccess())) {
    ERROR("{} failed to get in resting pose", robot.name());
  } else if (do_exclusion_zone_change &&
             !robot.getInSafePoseNearTarget(destination_.resource()->pose())) {
    ERROR("{} failed to get in resting pose", robot.name());
  } else if (do_exclusion_zone_change &&
             (!model_tray_access_manager.getAccessToModel(
                 destination_parent_name, timeout_))) {
    ERROR("{} failed to setup access constraints to target", robot.name());
  } else if (!robot.getToGraspingPoseHint(destination_.resource()->pose())) {
    ERROR("{} failed to get closer to target", robot.name());
  } else if (!robot.getInLandingSpot(destination_.resource()->pose()) ||
             !robot.gripperHasPartAttached()) {
    ERROR("{} failed to get to the destination landing pose with the part "
          "grasped",
          robot.name());
  } else if (!robot.placePartFromAbove(destination_.resource()->pose())) {
    ERROR("{} failed to place the part in the destination pose", robot.name());
  } else {
    result = RobotTaskOutcome::TASK_SUCCESS;
    ManagedLocus::TransferPartFromHereToThere(*source_.resource(),
                                              *destination_.resource());
    robot.getInLandingSpot(destination_.resource()->pose());
    INFO("{} successfully moved the part from {} to {}", robot.name(),
         source_.resource()->pose(), destination_.resource()->pose());
  }

  // if we failed the task at some point, we lost certainty about where the
  // source part is
  if (result != RobotTaskOutcome::TASK_SUCCESS) {
    // increase the difficulty of the piece
    source_.resource()->correctDifficulty(1);
  }

  // try to get in a resting pose to remove the robot from the way
  robot.dropPartWhereYouStand();
  robot.getInSafePose();
  model_tray_access_manager.releaseAccess();
  return result;
}

void PickAndPlaceTask::halt() {
  auto &robot = *robot_.resource();
  robot.cancelAction();
}

} // namespace tijcore
