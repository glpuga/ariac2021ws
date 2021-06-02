/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// standard library
#include <chrono>

// tijcore
#include <logger/logger.hpp>
#include <tijcore/tasks/ModelTrayAccessSpaceManager.hpp>
#include <tijcore/tasks/PickAndTwistPartTask.hpp>

namespace tijcore {

namespace {

constexpr std::chrono::seconds timeout_{120};
}

PickAndTwistPartTask::PickAndTwistPartTask(
    const ResourceManagerInterface::SharedPtr &resource_manager,
    ResourceManagerInterface::ManagedLocusHandle &&target,
    ResourceManagerInterface::ManagedLocusHandle &&destination,
    ResourceManagerInterface::PickAndPlaceRobotHandle &&robot)
    : resource_manager_{resource_manager}, target_{std::move(target)},
      destination_{std::move(destination)}, robot_{std::move(robot)} {}

RobotTaskOutcome PickAndTwistPartTask::run() {
  auto &robot = *robot_.resource();
  RobotTaskOutcome result{RobotTaskOutcome::TASK_FAILURE};

  tijcore::PartTypeId part_type_id;
  {
    auto [part_type, broken] = target_.resource()->model();
    (void)broken;
    part_type_id = part_type.type();
  }

  ModelTrayAccessSpaceManager model_tray_access_manager(*resource_manager_,
                                                        robot);

  // clear exclusion zones to enable movement to a safe spot regardless of where
  // we are located
  model_tray_access_manager.clearAllExclusionZones();

  // TODO(glpuga) generalize this code so that we can rotate pieces with other
  // rotations.

  const auto source_parent_name = target_.resource()->parentName();
  const auto destination_parent_name = destination_.resource()->parentName();

  // if we don't change exclusion zones, we can skip some time-consuming steps
  const bool do_exclusion_zone_change =
      resource_manager_->getContainerExclusionZoneId(source_parent_name) !=
      resource_manager_->getContainerExclusionZoneId(destination_parent_name);

  if (!robot.getInSafePoseNearTarget(target_.resource()->pose()) ||
      !model_tray_access_manager.releaseAccess()) {
    ERROR("{} failed to get in resting pose", robot.name());
  } else if ((do_exclusion_zone_change &&
              !model_tray_access_manager.getAccessToModel(source_parent_name,
                                                          timeout_)) ||
             (!do_exclusion_zone_change &&
              !model_tray_access_manager.getAccessToModel(
                  source_parent_name, destination_parent_name, timeout_))) {
    ERROR("{} failed to setup access constraints to target", robot.name());
  } else if (!robot.getToGraspingPoseHint(target_.resource()->pose())) {
    ERROR("{} failed to get closer to target", robot.name());
  } else if (!robot.getInLandingSpot(target_.resource()->pose())) {
    ERROR("{} failed to get into the landing pose prior to grasping",
          robot.name());
  } else if (!robot.graspPartFromAbove(target_.resource()->pose(),
                                       part_type_id)) {
    ERROR("{} failed to grasp the part form the surface", robot.name());
  } else if (!robot.getInLandingSpot(target_.resource()->pose()) ||
             !robot.gripperHasPartAttached()) {
    ERROR("{} failed to get into the landing pose post grasping with the part "
          "grasped",
          robot.name());
  } else if (do_exclusion_zone_change &&
             (!robot.getInSafePoseNearTarget(target_.resource()->pose()) ||
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
    // this is where it gets interesting...
  } else {
    ManagedLocus::TransferPartFromHereToThere(*target_.resource(),
                                              *destination_.resource());

    // hackish way to compensate for the difference in which heights are handled
    // for empty spaces (like destination) and parts (like target), because
    // cameras report part height at about mid-height, while empty spaces
    // have locations on the surface.
    destination_.resource()->pose().position().vector().z() =
        target_.resource()->pose().position().vector().z();

    if (!robot.twistPartInPlace(destination_.resource()->pose(),
                                part_type_id) ||
        !robot.dropPartWhereYouStand()) {
      ERROR("{} failed to twist the part in place (first twist)", robot.name());
    } else if ((!robot.getInLandingSpot(destination_.resource()->pose()))) {
      ERROR("{} failed to get in the landing pose (second twist)",
            robot.name());
    } else if ((!robot.graspPartFromAbove(destination_.resource()->pose(),
                                          part_type_id))) {
      ERROR("{} failed to grasp the part form the surface (second twist)",
            robot.name());
    } else if ((!robot.getInLandingSpot(destination_.resource()->pose()) ||
                !robot.gripperHasPartAttached())) {
      ERROR("{} failed to get back into the landing pose with the part (second "
            "twist)",
            robot.name());
    } else if ((!robot.twistPartInPlace(destination_.resource()->pose(),
                                        part_type_id) ||
                !robot.dropPartWhereYouStand())) {
      ERROR("{} failed to twist the part in place (second twist)",
            robot.name());
    } else {
      result = RobotTaskOutcome::TASK_SUCCESS;
      robot.getInLandingSpot(destination_.resource()->pose());
      INFO("{} successfully flipped the part from {} to {}", robot.name(),
           target_.resource()->pose(), destination_.resource()->pose());
    }
  }

  // if we failed the task at some point, we lost certainty about where the
  // source part is
  if (result != RobotTaskOutcome::TASK_SUCCESS) {
    // increase the difficulty of the piece
    destination_.resource()->correctDifficulty(1);
  }

  // try to get in a resting pose to remove the robot from the way
  robot.dropPartWhereYouStand();
  robot.getInSafePose();
  model_tray_access_manager.releaseAccess();
  return result;
}

void PickAndTwistPartTask::halt() {
  auto &robot = *robot_.resource();
  robot.cancelAction();
}

} // namespace tijcore
