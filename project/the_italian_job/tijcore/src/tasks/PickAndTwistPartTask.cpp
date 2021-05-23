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
  using TwistDirection = PickAndPlaceRobotInterface::TwistDirection;

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

  // TODO(glpuga) generaliza this code so that we can rotate pieces with other
  // rotations.
  const int32_t angular_twist = 180;

  TwistDirection first_twist{TwistDirection::left};
  TwistDirection second_twist{TwistDirection::left};
  bool do_second_twist{false};

  {
    // + 360 to allow for a range from -360 to 360
    const auto sides_to_turn = ((360 + angular_twist + 45) / 90) % 4;

    switch (sides_to_turn) {
    case 0:
      // WTF
      ERROR("Requested a Pick and Flip for a zero degree flip!");
      return RobotTaskOutcome::TASK_FAILURE;
      break;
    case 1:
      first_twist = TwistDirection::left;
      do_second_twist = false;
      break;
    case 2:
      first_twist = TwistDirection::left;
      second_twist = TwistDirection::left;
      do_second_twist = true;
      break;
    case 3:
      first_twist = TwistDirection::right;
      do_second_twist = false;
      break;
    }
  }

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
  } else if (!robot.twistPartInPlace(part_.resource()->pose(), first_twist) ||
             !robot.dropPartWhereYouStand()) {
    ERROR("{} failed to twist the part in place (first twist)", robot.name());
  } else if (do_second_twist &&
             (!robot.getInLandingSpot(part_.resource()->pose()))) {
    ERROR("{} failed to get in the landing pose (second twist)", robot.name());
  } else if (do_second_twist && (!robot.graspPartFromAbove(
                                    part_.resource()->pose(), part_type_id))) {
    ERROR("{} failed to grasp the part form the surface (second twist)",
          robot.name());
  } else if (do_second_twist &&
             (!robot.getInLandingSpot(part_.resource()->pose()) ||
              !robot.gripperHasPartAttached())) {
    ERROR("{} failed to get back into the landing pose with the part (second "
          "twist)",
          robot.name());
  } else if (do_second_twist &&
             (!robot.twistPartInPlace(part_.resource()->pose(), second_twist) ||
              !robot.dropPartWhereYouStand())) {
    ERROR("{} failed to twist the part in place (second twist)", robot.name());
  } else {
    result = RobotTaskOutcome::TASK_SUCCESS;
    // TODO(glpuga) I should update the rotation of the part here, but as first
    // approximation let the camera sort it out (THAT WILL FAIL IF WE ARE IN
    // BLACKOUT)
    INFO("{} successfully twisted the part at {} by {} degrees", robot.name(),
         part_.resource()->pose(), angular_twist);
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
