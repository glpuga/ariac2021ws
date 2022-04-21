/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// standard library
#include <utility>

// tijcore
#include <tijcore/tasking/RemoveBrokenPartTask.hpp>
#include <tijlogger/logger.hpp>

namespace tijcore
{
namespace
{
constexpr std::chrono::seconds timeout_{ 120 };
}

RemoveBrokenPartTask::RemoveBrokenPartTask(
    const ResourceManagerInterface::SharedPtr& resource_manager, const Toolbox::SharedPtr& toolbox,
    ResourceManagerInterface::ManagedLocusHandle&& target,
    ResourceManagerInterface::PickAndPlaceRobotHandle&& robot)
  : resource_manager_{ resource_manager }
  , toolbox_{ toolbox }
  , target_{ std::move(target) }
  , robot_{ std::move(robot) }
{
}

RobotTaskOutcome RemoveBrokenPartTask::run()
{
  // auto& robot = *robot_.resource();
  // auto scene = toolbox_->getSceneConfigReader();
  RobotTaskOutcome result{ RobotTaskOutcome::TASK_FAILURE };

  // tijcore::PartTypeId part_type_id;
  // {
  //   const auto part_type = target_.resource()->partId();
  //   part_type_id = part_type.type();
  // }

  // const auto target_parent_name = target_.resource()->parentName();

  // if (!robot.getInSafePoseNearTarget(target_.resource()->pose()))
  // {
  //   ERROR("{} failed to get in resting pose", robot.name());
  // }
  // else if (!robot.getToGraspingPoseHint(target_.resource()->pose()))
  // {
  //   ERROR("{} failed to get closer to target", robot.name());
  // }
  // else if (!robot.getInLandingSpot(target_.resource()->pose()))
  // {
  //   ERROR("{} failed to get into the approximation pose to remove a broken part", robot.name());
  // }
  // else if (!robot.graspPartFromAbove(target_.resource()->pose(), part_type_id))
  // {
  //   ERROR("{} failed to pick up the broken part while trying to remove it", robot.name());
  // }
  // else if (!robot.getInSafePoseNearTarget(target_.resource()->pose()) ||
  //          !robot.gripperHasPartAttached())
  // {
  //   ERROR("{} failed to get the broken part ready for transport to the bucket", robot.name());
  // }
  // else if (!robot.getToGraspingPoseHint(scene->getDropBucketPose()))
  // {
  //   ERROR("{} failed to get closer to target", robot.name());
  // }
  // else if (!robot.getInLandingSpot(scene->getDropBucketPose()))
  // {
  //   ERROR(
  //       "{} failed to get in the approximation pose to drop the broken "
  //       "part into the bucket",
  //       robot.name());
  // }
  // else if (!robot.dropPartWhereYouStand())
  // {
  //   ERROR("{} failed to drop the broken part into the bucket", robot.name());
  // }
  // else
  // {
  //   result = RobotTaskOutcome::TASK_SUCCESS;
  //   // create an empty pose anywhere, we just need it to make the part info
  //   // dissapear
  //   auto limbo = ManagedLocus::CreateEmptyLocus("drop_bucket", scene->getDropBucketPose());
  //   ManagedLocus::TransferPartFromHereToThere(*target_.resource(), limbo);
  //   INFO(
  //       "{} successfully removed a broken part and placed it into the "
  //       "bucket",
  //       robot.name());
  // }

  // // if we failed the task at some point, we lost certainty about where the
  // // source part is
  // if (result != RobotTaskOutcome::TASK_SUCCESS)
  // {
  //   *target_.resource() = ManagedLocus::CreateEmptyLocus(target_.resource()->parentName(),
  //                                                        target_.resource()->pose());
  // }

  // // try to get in a resting pose to remove the robot from the way
  // robot.dropPartWhereYouStand();
  // robot.getInSafePose();
  return result;
}

void RemoveBrokenPartTask::halt()
{
  auto& robot = *robot_.resource();
  robot.abortCurrentAction();
}

}  // namespace tijcore
