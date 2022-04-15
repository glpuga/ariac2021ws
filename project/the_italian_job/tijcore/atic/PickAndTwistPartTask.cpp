/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// standard library
#include <chrono>
#include <utility>

// tijcore
#include <tijcore/tasking/PickAndTwistPartTask.hpp>
#include <tijlogger/logger.hpp>

namespace tijcore
{
namespace
{
constexpr std::chrono::seconds timeout_{ 120 };
}

PickAndTwistPartTask::PickAndTwistPartTask(
    const ResourceManagerInterface::SharedPtr& resource_manager,
    ResourceManagerInterface::ManagedLocusHandle&& target,
    ResourceManagerInterface::ManagedLocusHandle&& destination,
    ResourceManagerInterface::PickAndPlaceRobotHandle&& robot)
  : resource_manager_{ resource_manager }
  , target_{ std::move(target) }
  , destination_{ std::move(destination) }
  , robot_{ std::move(robot) }
{
}

RobotTaskOutcome PickAndTwistPartTask::run()
{
  auto& robot = *robot_.resource();
  RobotTaskOutcome result{ RobotTaskOutcome::TASK_FAILURE };

  tijcore::PartTypeId part_type_id;
  {
    const auto part_type = target_.resource()->partId();
    part_type_id = part_type.type();
  }

  // TODO(glpuga) generalize this code so that we can rotate pieces with other
  // rotations.

  const auto source_parent_name = target_.resource()->parentName();
  const auto destination_parent_name = destination_.resource()->parentName();

  if (!robot.getInSafePoseNearTarget(target_.resource()->pose()))
  {
    ERROR("{} failed to get in resting pose", robot.name());
  }
  else if (!robot.getToGraspingPoseHint(target_.resource()->pose()))
  {
    ERROR("{} failed to get closer to target", robot.name());
  }
  else if (!robot.getInLandingSpot(target_.resource()->pose()))
  {
    ERROR("{} failed to get into the landing pose prior to grasping", robot.name());
  }
  else if (!robot.graspPartFromAbove(target_.resource()->pose(), part_type_id))
  {
    ERROR("{} failed to grasp the part form the surface", robot.name());
  }
  else if (!robot.getInLandingSpot(target_.resource()->pose()) || !robot.gripperHasPartAttached())
  {
    ERROR(
        "{} failed to get into the landing pose post grasping with the part "
        "grasped",
        robot.name());
  }
  else if (!robot.getToGraspingPoseHint(destination_.resource()->pose()))
  {
    ERROR("{} failed to get closer to target", robot.name());
  }
  else if (!robot.getInLandingSpot(destination_.resource()->pose()) ||
           !robot.gripperHasPartAttached())
  {
    ERROR(
        "{} failed to get to the destination landing pose with the part "
        "grasped",
        robot.name());
    // this is where it gets interesting...
  }
  else
  {
    ManagedLocus::TransferPartFromHereToThere(*target_.resource(), *destination_.resource());

    // hackish way to compensate for the difference in which heights are handled
    // for empty spaces (like destination) and parts (like target), because
    // cameras report part height at about mid-height, while empty spaces
    // have locations on the surface.
    destination_.resource()->pose().position().vector().z() =
        target_.resource()->pose().position().vector().z();

    if (!robot.twistPartInPlace(destination_.resource()->pose(), part_type_id) ||
        !robot.dropPartWhereYouStand())
    {
      ERROR("{} failed to twist the part in place (first twist)", robot.name());
    }
    else if ((!robot.getInLandingSpot(destination_.resource()->pose())))
    {
      ERROR("{} failed to get in the landing pose (second twist)", robot.name());
    }
    else if ((!robot.graspPartFromAbove(destination_.resource()->pose(), part_type_id)))
    {
      ERROR("{} failed to grasp the part form the surface (second twist)", robot.name());
    }
    else if ((!robot.getInLandingSpot(destination_.resource()->pose()) ||
              !robot.gripperHasPartAttached()))
    {
      ERROR(
          "{} failed to get back into the landing pose with the part (second "
          "twist)",
          robot.name());
    }
    else if ((!robot.twistPartInPlace(destination_.resource()->pose(), part_type_id) ||
              !robot.dropPartWhereYouStand()))
    {
      ERROR("{} failed to twist the part in place (second twist)", robot.name());
    }
    else
    {
      result = RobotTaskOutcome::TASK_SUCCESS;
      robot.getInLandingSpot(destination_.resource()->pose());
      INFO("{} successfully flipped the part from {} to {}", robot.name(),
           target_.resource()->pose(), destination_.resource()->pose());
    }
  }

  // try to get in a resting pose to remove the robot from the way
  robot.dropPartWhereYouStand();
  robot.getInSafePose();
  return result;
}

void PickAndTwistPartTask::halt()
{
  auto& robot = *robot_.resource();
  robot.cancelAction();
}

}  // namespace tijcore
