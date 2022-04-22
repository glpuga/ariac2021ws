/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// standard library
#include <chrono>
#include <thread>
#include <utility>

// tijcore
#include <tijcore/tasking/PickAndPlaceTask.hpp>
#include <tijlogger/logger.hpp>

namespace tijcore
{
PickAndPlaceTask::PickAndPlaceTask(const ResourceManagerInterface::SharedPtr& resource_manager,
                                   ResourceManagerInterface::ManagedLocusHandle&& source,
                                   ResourceManagerInterface::ManagedLocusHandle&& destination,
                                   ResourceManagerInterface::PickAndPlaceRobotHandle&& robot)
  : resource_manager_{ resource_manager }
  , source_{ std::move(source) }
  , destination_{ std::move(destination) }
  , robot_{ std::move(robot) }
{
}

RobotTaskOutcome PickAndPlaceTask::run()
{
  RobotTaskOutcome result{ RobotTaskOutcome::TASK_FAILURE };

  auto& robot = *robot_.resource();

  tijcore::PartTypeId part_type_id;
  const auto source_pose = source_.resource()->pose();

  if (source_.resource()->isLocusWithPart())
  {
    const auto part_type = source_.resource()->qualifiedPartInfo().part_type;
    part_type_id = part_type.type();
  }
  else if (source_.resource()->isLocusWithMovableTray())
  {
    // TODO(glpuga) hacky way to implement moving trays using flat pieces until
    // we have a better solution
    part_type_id = tijcore::PartTypeId::regulator;
  }
  else
  {
    return RobotTaskOutcome::TASK_FAILURE;
  }

  if (!robot.getInSafePoseNearTarget(source_pose))
  {
    ERROR("{} failed to get in resting pose", robot.name());
  }
  else if (!robot.getInLandingSpot(source_pose))
  {
    ERROR("{} failed to get into the landing pose prior to grasping", robot.name());
  }
  else if (!robot.contactPartFromAboveAndGrasp(source_pose, part_type_id))
  {
    ERROR("{} failed to grasp the part form the surface", robot.name());
  }
  else if (!robot.getInLandingSpot(source_pose))
  {
    ERROR("{} failed to get into the landing pose prior to grasping", robot.name());
  }
  else if (!robot.getArmInRestingPose() || !robot.gripperHasPartAttached())
  {
    ERROR(
        "{} failed to get into the landing pose post grasping with the part "
        "grasped",
        robot.name());
  }
  else if (!robot.getInSafePoseNearTarget(destination_.resource()->pose()))
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
  }
  else if (!robot.placePartFromAbove(destination_.resource()->pose(), part_type_id))
  {
    ERROR("{} failed to place the part in the destination pose", robot.name());
  }
  else
  {
    result = RobotTaskOutcome::TASK_SUCCESS;
    ManagedLocus::TransferPartFromHereToThere(*source_.resource(), *destination_.resource());
    robot.getInSafePoseNearTarget(destination_.resource()->pose());
    INFO("{} successfully moved the part from {} to {}", robot.name(), source_pose,
         destination_.resource()->pose());
  }

  // try to get in a resting pose to remove the robot from the way
  robot.turnOffGripper();
  robot.getArmInRestingPose();

  if (result != RobotTaskOutcome::TASK_SUCCESS)
  {
    {
      source_.release();
      destination_.release();
    }
  }

  return result;
}

void PickAndPlaceTask::halt()
{
  auto& robot = *robot_.resource();
  robot.abortCurrentAction();
}

}  // namespace tijcore
