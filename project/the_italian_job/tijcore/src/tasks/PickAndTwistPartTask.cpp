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
    ResourceManagerInterface::ManagedLocusHandle &&part,
    ResourceManagerInterface::ManagedLocusHandle &&destination,
    ResourceManagerInterface::PickAndPlaceRobotHandle &&robot)
    : resource_manager_{resource_manager}, part_{std::move(part)},
      destination_{std::move(destination)}, robot_{std::move(robot)} {}

RobotTaskOutcome PickAndTwistPartTask::run() {
  auto &robot = *robot_.resource();
  RobotTaskOutcome result{RobotTaskOutcome::TASK_FAILURE};

  tijcore::PartTypeId part_type_id;
  {
    auto [part_type, broken] = part_.resource()->model();
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

  const auto source_parent_name = part_.resource()->parentName();

  if (!robot.getInSafePoseNearTarget(part_.resource()->pose()) ||
      !model_tray_access_manager.releaseAccess()) {
    ERROR("{} failed to get in a safe pose near target", robot.name());
  } else if (!model_tray_access_manager.getAccessToModel(source_parent_name,
                                                         timeout_)) {
    ERROR("{} failed to get access to exlusion zone", robot.name());
  } else if (!robot.getToGraspingPoseHint(part_.resource()->pose())) {
    ERROR("{} failed to get in graping hint pose", robot.name());
  } else if (!robot.getInLandingSpot(part_.resource()->pose())) {
    ERROR("{} failed to get in the landing pose (first twist)", robot.name());
  } else if (!robot.graspPartFromAbove(part_.resource()->pose(),
                                       part_type_id)) {
    ERROR("{} failed to grasp the part form the surface (first twist)",
          robot.name());
  } else if ((!robot.getInLandingSpot(part_.resource()->pose()) ||
              !robot.gripperHasPartAttached())) {
    ERROR("{} failed to get back into the landing pose with the part (first "
          "twist)",
          robot.name());
    // this is where it gets interesting...
  } else if (!robot.twistPartInPlace(part_.resource()->pose(), part_type_id) ||
             !robot.dropPartWhereYouStand()) {
    ERROR("{} failed to twist the part in place (first twist)", robot.name());
  } else if ((!robot.getInLandingSpot(part_.resource()->pose()))) {
    ERROR("{} failed to get in the landing pose (second twist)", robot.name());
  } else if ((!robot.graspPartFromAbove(part_.resource()->pose(),
                                        part_type_id))) {
    ERROR("{} failed to grasp the part form the surface (second twist)",
          robot.name());
  } else if ((!robot.getInLandingSpot(part_.resource()->pose()) ||
              !robot.gripperHasPartAttached())) {
    ERROR("{} failed to get back into the landing pose with the part (second "
          "twist)",
          robot.name());
  } else if ((!robot.twistPartInPlace(part_.resource()->pose(), part_type_id) ||
              !robot.dropPartWhereYouStand())) {
    ERROR("{} failed to twist the part in place (second twist)", robot.name());
  } else {
    result = RobotTaskOutcome::TASK_SUCCESS;
    INFO("{} successfully flipped the part at {}", robot.name(),
         part_.resource()->pose());
  }

  // if we failed the task at some point, we lost certainty about where the
  // source part is
  if (result != RobotTaskOutcome::TASK_SUCCESS) {
    // increase the difficulty of the piece
    part_.resource()->correctDifficulty(1);
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
