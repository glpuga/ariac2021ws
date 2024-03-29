/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// standard library
#include <utility>

// tijcore
#include <tijcore/tasking/RemoveBrokenPartTask.hpp>
#include <tijcore/utils/PayloadEnvelope.hpp>
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

  auto frame_transformer = toolbox_->getFrameTransformer();
  const auto target_pose_in_world_frame =
      frame_transformer->transformPoseToFrame(target_.resource()->pose(), scene->getWorldFrameId());

  const auto offset_to_top =
      tijcore::PayloadEnvelope::offsetToTop(part_type_id, target_pose_in_world_frame.rotation());

  const auto target_parent_name = target_.resource()->parentName();

  const auto required_gripper_type = tijcore::GripperTypeId::gripper_part;
  const auto initial_tool_type = robot.getRobotGripperToolType();

  const auto payload_envelope_info = tijcore::PayloadEnvelope::makeEnvelope(part_type_id);

  const auto ee_to_payload_transform = robot.calculatePayloadIntoEndEffectorTransform(
      robot.calculateVerticalGripEndEffectorPose(target_pose_in_world_frame, offset_to_top),
      target_pose_in_world_frame);

  if ((initial_tool_type != required_gripper_type) &&
      !robot.getRobotInSafePoseNearTarget(scene->getGripperToolSwappingTablePose()))
  {
    ERROR("{} was unable to go to the gripper swapping area to change from tool type from {} to {}",
          robot.getRobotName(), initial_tool_type, required_gripper_type);
  }
  else if ((initial_tool_type != required_gripper_type) &&
           !robot.setRobotGripperToolType(required_gripper_type))
  {
    ERROR("{} was unable to switch the gripper tool type from {} to {}", robot.getRobotName(),
          initial_tool_type, required_gripper_type);
  }
  else if ((initial_tool_type != required_gripper_type) &&
           (robot.getRobotGripperToolType() != required_gripper_type))
  {
    ERROR(
        "{} was unable to check that the tool type is the one needed for the operation, from {} to "
        "{}",
        robot.getRobotName(), initial_tool_type, required_gripper_type);
  }
  // else if (!robot.getRobotInSafePoseNearTarget(target_pose_in_world_frame))
  // {
  //   ERROR("{} failed to get in resting pose", robot.getRobotName());
  // }
  else if (!robot.getGripperIn3DPoseJoinSpace(
               robot.calculateVerticalLandingPose(target_pose_in_world_frame, offset_to_top)))
  {
    ERROR("{} failed to get into the approximation pose to remove a broken part",
          robot.getRobotName());
  }
  else if (!robot.contactPartFromAboveAndGrasp(robot.calculateVerticalGripEndEffectorPose(
               target_pose_in_world_frame, offset_to_top)))
  {
    ERROR("{} failed to pick up the broken part while trying to remove it", robot.getRobotName());
  }
  else if (!robot.getRobotInSafePoseNearTarget(target_pose_in_world_frame) ||
           !robot.getRobotGripperAttachementState())
  {
    ERROR("{} failed to get the broken part ready for transport to the bucket",
          robot.getRobotName());
  }
  else if (!robot.setRobotGripperPayloadEnvelope(payload_envelope_info, ee_to_payload_transform))
  {
    ERROR("{} failed to enable the payload obstacle envelope", robot.getRobotName());
  }
  else if (!robot.getRobotInSafePoseNearTarget(scene->getDropBucketPose()))
  {
    ERROR("{} failed to get closer to target", robot.getRobotName());
  }
  else if (!robot.getGripperIn3DPoseJoinSpace(
               robot.calculateVerticalLandingPose(scene->getDropBucketPose(), offset_to_top)))
  {
    ERROR(
        "{} failed to get in the approximation pose to drop the broken "
        "part into the bucket",
        robot.getRobotName());
  }
  else if (!robot.removeRobotGripperPayloadEnvelope())
  {
    ERROR("{} failed to disable the payload obstacle envelope", robot.getRobotName());
  }
  else if (!robot.getRobotGripperOff())
  {
    ERROR("{} failed to drop the broken part into the bucket", robot.getRobotName());
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
        robot.getRobotName());
  }

  // if we failed the task at some point, we lost certainty about where the
  // source part is
  if (result != RobotTaskOutcome::TASK_SUCCESS)
  {
    *target_.resource() = ManagedLocus::CreateEmptyLocus(target_.resource()->parentName(),
                                                         target_pose_in_world_frame);
  }

  // try to get in a resting pose to remove the robot from the way
  robot.getRobotGripperOff();
  robot.getRobotArmInRestingPose();
  return result;
}

void RemoveBrokenPartTask::halt()
{
  auto& robot = *robot_.resource();
  robot.abortCurrentAction();
}

}  // namespace tijcore
