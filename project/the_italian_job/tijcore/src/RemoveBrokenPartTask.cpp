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
  RobotTaskOutcome result{ RobotTaskOutcome::TASK_FAILURE };

  auto& robot = *robot_.resource();
  auto scene = toolbox_->getSceneConfigReader();

  tijcore::PartTypeId part_type_id;
  {
    const auto part_type = target_.resource()->qualifiedPartInfo().part_type;
    part_type_id = part_type.type();
  }

  const auto target_parent_name = target_.resource()->parentName();

  const auto required_gripper_type = tijcore::GripperTypeId::gripper_part;
  const auto initial_tool_type = robot.getGripperToolType();

  if ((initial_tool_type != required_gripper_type) &&
      !robot.getInSafePoseNearTarget(scene->getGripperToolSwappingTablePose()))
  {
    ERROR("{} was unable to go to the gripper swapping area to change from tool type from {} to {}",
          robot.name(), initial_tool_type, required_gripper_type);
  }
  else if ((initial_tool_type != required_gripper_type) &&
           !robot.setGripperToolType(required_gripper_type))
  {
    ERROR("{} was unable to switch the gripper tool type from {} to {}", robot.name(),
          initial_tool_type, required_gripper_type);
  }
  else if ((initial_tool_type != required_gripper_type) &&
           (robot.getGripperToolType() != required_gripper_type))
  {
    ERROR(
        "{} was unable to check that the tool type is the one needed for the operation, from {} to "
        "{}",
        robot.name(), initial_tool_type, required_gripper_type);
  }
  // else if (!robot.getInSafePoseNearTarget(target_.resource()->pose()))
  // {
  //   ERROR("{} failed to get in resting pose", robot.name());
  // }
  else if (!robot.getInLandingSpot(target_.resource()->pose()))
  {
    ERROR("{} failed to get into the approximation pose to remove a broken part", robot.name());
  }
  else if (!robot.contactPartFromAboveAndGrasp(target_.resource()->pose(), part_type_id))
  {
    ERROR("{} failed to pick up the broken part while trying to remove it", robot.name());
  }
  else if (!robot.getInSafePoseNearTarget(target_.resource()->pose()) ||
           !robot.gripperHasPartAttached())
  {
    ERROR("{} failed to get the broken part ready for transport to the bucket", robot.name());
  }
  else if (!robot.getInSafePoseNearTarget(scene->getDropBucketPose()))
  {
    ERROR("{} failed to get closer to target", robot.name());
  }
  else if (!robot.getInLandingSpot(scene->getDropBucketPose()))
  {
    ERROR(
        "{} failed to get in the approximation pose to drop the broken "
        "part into the bucket",
        robot.name());
  }
  else if (!robot.turnOffGripper())
  {
    ERROR("{} failed to drop the broken part into the bucket", robot.name());
  }
  else
  {
    result = RobotTaskOutcome::TASK_SUCCESS;
    // create an empty pose anywhere, we just need it to make the part info
    // dissapear
    auto limbo = ManagedLocus::CreateEmptyLocus("drop_bucket", scene->getDropBucketPose());
    ManagedLocus::TransferPartFromHereToThere(*target_.resource(), limbo);
    INFO(
        "{} successfully removed a broken part and placed it into the "
        "bucket",
        robot.name());
  }

  // if we failed the task at some point, we lost certainty about where the
  // source part is
  if (result != RobotTaskOutcome::TASK_SUCCESS)
  {
    *target_.resource() = ManagedLocus::CreateEmptyLocus(target_.resource()->parentName(),
                                                         target_.resource()->pose());
  }

  // try to get in a resting pose to remove the robot from the way
  robot.turnOffGripper();
  robot.getArmInRestingPose();
  return result;
}

void RemoveBrokenPartTask::halt()
{
  auto& robot = *robot_.resource();
  robot.abortCurrentAction();
}

}  // namespace tijcore
