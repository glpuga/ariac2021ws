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
namespace
{
constexpr std::chrono::seconds timeout_{ 120 };

constexpr std::chrono::seconds sleep_interval_{ 1 };

}  // namespace

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
  auto& robot = *robot_.resource();
  RobotTaskOutcome result{ RobotTaskOutcome::TASK_FAILURE };

  tijcore::PartTypeId part_type_id;
  {
    const auto part_type = source_.resource()->partId();
    part_type_id = part_type.type();
  }

  const auto source_parent_name = source_.resource()->parentName();
  const auto destination_parent_name = destination_.resource()->parentName();

  if (!robot.getInSafePoseNearTarget(source_.resource()->pose()))
  {
    ERROR("{} failed to get in resting pose", robot.name());
  }
  else if (!robot.getToGraspingPoseHint(source_.resource()->pose()))
  {
    ERROR("{} failed to get closer to target", robot.name());
  }
  else if (!robot.getInLandingSpot(source_.resource()->pose()))
  {
    ERROR("{} failed to get into the landing pose prior to grasping", robot.name());
  }
  else if (!robot.graspPartFromAbove(source_.resource()->pose(), part_type_id))
  {
    ERROR("{} failed to grasp the part form the surface", robot.name());
  }
  else if (!robot.getInLandingSpot(source_.resource()->pose()) || !robot.gripperHasPartAttached())
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
  }
  else if (!robot.placePartFromAbove(destination_.resource()->pose(), part_type_id))
  {
    ERROR("{} failed to place the part in the destination pose", robot.name());
  }
  else
  {
    result = RobotTaskOutcome::TASK_SUCCESS;
    ManagedLocus::TransferPartFromHereToThere(*source_.resource(), *destination_.resource());
    robot.getInLandingSpot(destination_.resource()->pose());
    INFO("{} successfully moved the part from {} to {}", robot.name(), source_.resource()->pose(),
         destination_.resource()->pose());
  }

  // try to get in a resting pose to remove the robot from the way
  robot.dropPartWhereYouStand();
  robot.getInSafePose();

  if (result != RobotTaskOutcome::TASK_SUCCESS)
  {
    // release the loci, but not the robot, to other robots
    // the opportunity to give it a try.
    {
      auto destroy_src{ std::move(source_) };
      auto destroy_dst{ std::move(destination_) };
      (void)destroy_src;
      (void)destroy_dst;
    }
    WARNING(
        "Task failed, will release resource but hold the robot to allow "
        "other robots to give the task a try");
    std::this_thread::sleep_for(sleep_interval_);
  }

  return result;
}

void PickAndPlaceTask::halt()
{
  auto& robot = *robot_.resource();
  robot.cancelAction();
}

}  // namespace tijcore
