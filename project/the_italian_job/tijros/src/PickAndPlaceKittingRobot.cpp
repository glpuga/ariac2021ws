/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <chrono>
#include <map>
#include <string>
#include <vector>

// ros
#include <ros/ros.h>

// tijcore
#include <tijlogger/logger.hpp>
#include <tijmath/math_utilities.hpp>
#include <tijros/PickAndPlaceKittingRobot.hpp>

namespace tijros
{
using tijmath::utils::angles::degreesToRadians;

PickAndPlaceKittingRobot::PickAndPlaceKittingRobot(const tijcore::Toolbox::SharedPtr& toolbox)
  : frame_transformer_{ toolbox->getFrameTransformer() }
  , robot_actuator_{ toolbox->getRobotActuator() }
{
}

bool PickAndPlaceKittingRobot::getRobotHealthState() const
{
  auto health_status = robot_actuator_->getRobotHealthStatus();
  return health_status.kitting_robot_enabled;
}

std::string PickAndPlaceKittingRobot::getRobotName() const
{
  return "kitting";
}

std::string PickAndPlaceKittingRobot::getRobotPlanningGroup() const
{
  return "kitting_arm";
}

std::string PickAndPlaceKittingRobot::getRobotEndEffectorLinkName() const
{
  return "ee_link";
}

bool PickAndPlaceKittingRobot::getRobotGripperAttachementState() const
{
  auto gripper_state = robot_actuator_->getKittingGripperState();
  return gripper_state.attached;
}

void PickAndPlaceKittingRobot::setRobotGripperState(const bool state) const
{
  robot_actuator_->setKittingGripperSuction(state);
}

void PickAndPlaceKittingRobot::patchJointStateValuesForArmInRestingPose(
    std::vector<double>& joint_states) const
{
  if (joint_states.size() != 7)
  {
    WARNING("The size ({}) of the joint vector for {} is unexpected...", joint_states.size(),
            getRobotName());
  }
  // these correspond to the positions of the rest position of the arm,
  // excluding the linear rail

  // const auto sign = (joint_states[0] < 0) ? -1.0 : 1.0;
  // joint_states[1] = 1.57 * sign;

  joint_states[2] = -1.507;
  joint_states[3] = 2.216;
  joint_states[4] = 3.92;
  joint_states[5] = -1.507;
  joint_states[6] = 0;
}

void PickAndPlaceKittingRobot::patchJointStateValuesToFaceTarget(
    std::vector<double>& joint_states, const tijmath::RelativePose3& pose,
    const tijmath::RelativePose3& aim) const
{
  if (joint_states.size() != 7)
  {
    WARNING("The size ({}) of the joint vector for {} is unexpected...", joint_states.size(),
            getRobotName());
  }

  const auto prev_orientation = joint_states[1];
  // hacky, last week of competition code reuse. This changes orientation, that's why I
  // saved the orientation above first.
  patchJointStateValuesToGoTo2DPose(joint_states, pose);

  const auto pose_in_world = frame_transformer_->transformPoseToFrame(pose, "world");
  const auto aim_in_world = frame_transformer_->transformPoseToFrame(aim, "world");

  const auto pointing_vector = aim_in_world.position().vector() - pose_in_world.position().vector();

  auto closest_angular_alias = [](const double next, const double prev) {
    const auto diff = std::fmod(next - prev + 2 * M_PI, 4 * M_PI) - 2 * M_PI;
    return std::fmod(prev + diff, 2 * M_PI);
  };

  // ensure the minimum movement to get in the desired direction
  const auto necessary_orientation = std::atan2(pointing_vector.y(), pointing_vector.x());

  joint_states[1] = closest_angular_alias(necessary_orientation, prev_orientation);
  ERROR("Kitting being set to {}, prev {}", joint_states[1] * 180 / 3.1415,
        prev_orientation * 180 / 3.1415);
}

void PickAndPlaceKittingRobot::patchJointStateValuesToGetCloseToTargetPose(
    std::vector<double>& joint_states, const tijmath::RelativePose3& target) const
{
  patchJointStateValuesToGoTo2DPose(joint_states, target);
}

void PickAndPlaceKittingRobot::patchJointStateValuesToGoTo2DPose(
    std::vector<double>& joint_states, const tijmath::RelativePose3& target) const
{
  if (joint_states.size() != 7)
  {
    WARNING("The size ({}) of the joint vector for {} is unexpected...", joint_states.size(),
            getRobotName());
  }

  // TODO(glpuga): this should be generalized so that "world" is not hardcoded
  const auto target_in_world = frame_transformer_->transformPoseToFrame(target, "world");
  joint_states[0] = target_in_world.position().vector().y();

  // bias the location so that the robot is in a slight angle to grasp
  const auto part_in_belt = target_in_world.position().vector().x() > -1;
  if (part_in_belt)
  {
    joint_states[0] += 0.25;
    joint_states[1] = degreesToRadians(-90);
  }
  else
  {
    // const auto sign = (joint_states[0] < 0) ? -1.0 : 1.0;
    joint_states[1] = degreesToRadians(-90);
  }
}

bool PickAndPlaceKittingRobot::setGripperToolTypeImpl(const tijcore::GripperTypeId new_type) const
{
  if (new_type != tijcore::GripperTypeId::gripper_part)
  {
    ERROR("The gripper type type {} is not supported by the kitting robot...", new_type);
    return false;
  }
  WARNING("Requested a change of gripper tool to the kitting robot, with no effect");
  return true;
}

tijcore::GripperTypeId PickAndPlaceKittingRobot::getGripperToolTypeImpl() const
{
  return tijcore::GripperTypeId::gripper_part;
}

bool PickAndPlaceKittingRobot::testIfRobotReachesPose(const tijmath::RelativePose3& target) const
{
  // TODO(glpuga): this should be generalized so that "world" is not hardcoded
  const auto target_in_world = frame_transformer_->transformPoseToFrame(target, "world");
  const auto target_in_world_x = target_in_world.position().vector().x();
  return (-2.65 < target_in_world_x) && (target_in_world_x < 0.0);
}

tijmath::RelativePose3 PickAndPlaceKittingRobot::getCurrentRobotPose() const
{
  return tijmath::RelativePose3{ "base_link", {}, {} };
}

tijmath::RelativePose3 PickAndPlaceKittingRobot::getCurrentEndEffectorPose() const
{
  return tijmath::RelativePose3{ getRobotEndEffectorLinkName(), {}, {} };
}

void PickAndPlaceKittingRobot::patchJointStateValuesForAlignedZeroWrist(
    std::vector<double>& joint_states) const
{
  if (joint_states.size() != 7)
  {
    WARNING("The size ({}) of the joint vector for {} is unexpected...", joint_states.size(),
            getRobotName());
  }
  joint_states[6] = degreesToRadians(0);
}

bool PickAndPlaceKittingRobot::beltSupportSetState(tijmath::RelativePose3& part_pose,
                                                   const double vertical_fraction,
                                                   const double time_interval)
{
  const std::vector<std::string> joint_names = {
    "linear_arm_actuator_joint",
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
  };

  const std::vector<double> value_up = {
    0.0,                     // linear_arm_actuator_joint
    degreesToRadians(-182),  // shoulder_pan_joint
    degreesToRadians(-124),  // shoulder_lift_joint
    degreesToRadians(-117),  // elbow_joint
    degreesToRadians(-29),   // wrist_1_joint
    degreesToRadians(+91),   // wrist_2_joint
    degreesToRadians(0),     // wrist_3_joint
  };

  const std::vector<double> value_down = {
    0.0,                     // linear_arm_actuator_joint
    degreesToRadians(-182),  // shoulder_pan_joint
    degreesToRadians(-138),  // shoulder_lift_joint
    degreesToRadians(-115),  // elbow_joint
    degreesToRadians(-17),   // wrist_1_joint
    degreesToRadians(+91),   // wrist_2_joint
    degreesToRadians(0),     // wrist_3_joint
  };

  const auto part_pose_in_world = frame_transformer_->transformPoseToFrame(part_pose, "world");

  const double x_world = part_pose_in_world.position().vector().x();

  if (std::abs(x_world) > 4.5)
  {
    ERROR("The part pose is outside of the boundaries of the conveyor belt: {}",
          part_pose_in_world);
    return false;
  }

  std::map<std::string, double> joint_states;

  for (size_t i = 0; i < joint_names.size(); ++i)
  {
    joint_states[joint_names[i]] = (value_down[i] - value_up[i]) * vertical_fraction + value_up[i];
  }

  joint_states["linear_arm_actuator_joint"] = x_world;

  auto& direct_control_manager = robot_actuator_->getKittingJointDirectControlManager();
  return direct_control_manager.setJointPosition(joint_states, time_interval);
}

bool PickAndPlaceKittingRobot::beltSupportGetSyncState() const
{
  auto& direct_control_manager = robot_actuator_->getKittingJointDirectControlManager();
  return direct_control_manager.inTrackingMode();
}

}  // namespace tijros
