/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// standard library
#include <chrono>
#include <thread>
#include <utility>

// tijcore
#include <tijcore/tasking/PickAndPlaceTask.hpp>
#include <tijcore/utils/PayloadEnvelop.hpp>
#include <tijlogger/logger.hpp>

namespace tijcore
{
PickAndPlaceTask::PickAndPlaceTask(const ResourceManagerInterface::SharedPtr& resource_manager,
                                   const Toolbox::SharedPtr& toolbox,
                                   ResourceManagerInterface::ManagedLocusHandle&& source,
                                   ResourceManagerInterface::ManagedLocusHandle&& destination,
                                   ResourceManagerInterface::PickAndPlaceRobotHandle&& robot)
  : resource_manager_{ resource_manager }
  , toolbox_{ toolbox }
  , source_{ std::move(source) }
  , destination_{ std::move(destination) }
  , robot_{ std::move(robot) }
{
}

RobotTaskOutcome PickAndPlaceTask::run()
{
  RobotTaskOutcome result{ RobotTaskOutcome::TASK_FAILURE };

  auto& robot = *robot_.resource();
  auto scene = toolbox_->getSceneConfigReader();

  double offset_to_top = 0.0;

  tijcore::GripperTypeId required_gripper_type;

  auto frame_transformer = toolbox_->getFrameTransformer();
  const auto source_pose_in_world_frame =
      frame_transformer->transformPoseToFrame(source_.resource()->pose(), scene->getWorldFrameId());

  if (source_.resource()->isLocusWithPart())
  {
    const auto part_type = source_.resource()->qualifiedPartInfo().part_type;
    const auto part_type_id = part_type.type();
    offset_to_top =
        tijcore::PayloadEnvelope::offsetToTop(part_type_id, source_pose_in_world_frame.rotation());
    required_gripper_type = tijcore::GripperTypeId::gripper_part;
  }
  else if (source_.resource()->isLocusWithMovableTray())
  {
    const auto movable_tray_id = source_.resource()->qualifiedMovableTrayInfo().tray_type;
    offset_to_top = tijcore::PayloadEnvelope::offsetToTop(movable_tray_id);
    required_gripper_type = tijcore::GripperTypeId::gripper_tray;
  }
  else
  {
    return RobotTaskOutcome::TASK_FAILURE;
  }

  const auto initial_tool_type = robot.getRobotGripperToolType();

  if ((initial_tool_type != required_gripper_type) &&
      !robot.getRobotTo2DPose(scene->getGripperToolSwappingTablePose()))
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
  else if (!robot.getRobotInSafePoseNearTarget(source_pose_in_world_frame))
  {
    ERROR("{} failed to get closer to target", robot.getRobotName());
  }
  else if (!robot.getGripperIn3DPoseJoinSpace(
               robot.calculateVerticalLandingPose(source_pose_in_world_frame)))
  {
    ERROR("{} failed to get into the landing pose prior to grasping", robot.getRobotName());
  }
  else if (!robot.contactPartFromAboveAndGrasp(source_pose_in_world_frame, offset_to_top))
  {
    ERROR("{} failed to grasp the part form the surface", robot.getRobotName());
  }
  else if (!robot.getGripperIn3DPoseJoinSpace(
               robot.calculateVerticalLandingPose(source_pose_in_world_frame)))
  {
    ERROR("{} failed to get into the landing pose prior to grasping", robot.getRobotName());
  }
  else if (!robot.getRobotGripperAttachementState())
  {
    ERROR(
        "{} failed to get into the landing pose post grasping with the part "
        "grasped",
        robot.getRobotName());
  }
  else if (!robot.getRobotInSafePoseNearTarget(destination_.resource()->pose()))
  {
    ERROR("{} failed to get closer to target", robot.getRobotName());
  }
  else if (!robot.getGripperIn3DPoseJoinSpace(
               robot.calculateVerticalLandingPose(destination_.resource()->pose())) ||
           !robot.getRobotGripperAttachementState())
  {
    ERROR(
        "{} failed to get to the destination landing pose with the part "
        "grasped",
        robot.getRobotName());
  }
  else if (!robot.getGripperIn3DPoseCartesianSpace(
               robot.calculateVerticalDropPose(destination_.resource()->pose(), offset_to_top)))
  {
    ERROR("{} failed to place the part in the destination pose", robot.getRobotName());
  }
  else
  {
    result = RobotTaskOutcome::TASK_SUCCESS;
    ManagedLocus::TransferPartFromHereToThere(*source_.resource(), *destination_.resource());
    robot.getRobotInSafePoseNearTarget(destination_.resource()->pose());
    INFO("{} successfully moved the part from {} to {}", robot.getRobotName(),
         source_pose_in_world_frame, destination_.resource()->pose());
  }

  // try to get in a resting pose to remove the robot from the way
  robot.getRobotGripperOff();
  robot.getRobotInSafePoseNearTarget(destination_.resource()->pose());

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
