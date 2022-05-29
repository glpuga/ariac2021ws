/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

// ros
#include <ros/ros.h>

// tijcore
#include <tijlogger/logger.hpp>
#include <tijmath/math_utilities.hpp>
#include <tijros/PickAndPlaceAssemblyRobot.hpp>

namespace tijros
{
namespace
{
// TODO(glpuga) move this into the scene configuration
constexpr char long_rail_frame_id_[] = "long_rail_1";
}  // namespace

using tijmath::utils::angles::degreesToRadians;

PickAndPlaceAssemblyRobot::PickAndPlaceAssemblyRobot(const tijcore::Toolbox::SharedPtr& toolbox)
  : frame_transformer_{ toolbox->getFrameTransformer() }
  , scene_config_{ toolbox->getSceneConfigReader() }
  , robot_actuator_{ toolbox->getRobotActuator() }
{
}

bool PickAndPlaceAssemblyRobot::getRobotHealthState() const
{
  auto health_status = robot_actuator_->getRobotHealthStatus();
  return health_status.assembly_robot_enabled;
}

std::string PickAndPlaceAssemblyRobot::getRobotName() const
{
  return "gantry";
}

std::string PickAndPlaceAssemblyRobot::getRobotPlanningGroup() const
{
  return "gantry_full";
}

std::string PickAndPlaceAssemblyRobot::getRobotEndEffectorLinkName() const
{
  return "gantry_arm_ee_link";
}

bool PickAndPlaceAssemblyRobot::getRobotGripperAttachementState() const
{
  auto gripper_state = robot_actuator_->getGantryGripperState();
  return gripper_state.attached;
}

void PickAndPlaceAssemblyRobot::setRobotGripperState(const bool state) const
{
  robot_actuator_->setGantryGripperSuction(state);
}

bool PickAndPlaceAssemblyRobot::setGripperToolTypeImpl(const tijcore::GripperTypeId new_type) const
{
  if (!robot_actuator_->setGantryGripperToolType(new_type))
  {
    return false;
  }
  while (robot_actuator_->getGantryGripperToolType() != new_type)
  {
    INFO("Waiting for gripper tool to be updated");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  INFO("Gantry gripper tool updated to {}", robot_actuator_->getGantryGripperToolType());
  return true;
}

tijcore::GripperTypeId PickAndPlaceAssemblyRobot::getGripperToolTypeImpl() const
{
  return robot_actuator_->getGantryGripperToolType();
}

void PickAndPlaceAssemblyRobot::patchJointStateValuesForArmInRestingPose(
    std::vector<double>& joint_states) const
{
  if (joint_states.size() != 9)
  {
    WARNING("The size ({}) of the joint vector for {} is unexpected...", joint_states.size(),
            getRobotName());
  }
  // Note that rail coordinates are relative to the rail, not to world

  joint_states[2] = 0.0;

  // resting "classic 2021"
  // joint_states[3] = degreesToRadians(180);
  // joint_states[4] = degreesToRadians(-60);
  // joint_states[5] = degreesToRadians(-90);
  // joint_states[6] = degreesToRadians(-30);
  // joint_states[7] = degreesToRadians(90);
  // joint_states[8] = degreesToRadians(0);

  // resting new
  joint_states[3] = degreesToRadians(180);
  joint_states[4] = degreesToRadians(-60);
  joint_states[5] = degreesToRadians(-120);
  joint_states[6] = degreesToRadians(0);
  joint_states[7] = degreesToRadians(90);
  joint_states[8] = degreesToRadians(0);
}

void PickAndPlaceAssemblyRobot::patchJointStateValuesToFaceTarget(
    std::vector<double>& joint_states, const tijmath::RelativePose3& pose,
    const tijmath::RelativePose3& aim) const
{
  if (joint_states.size() != 9)
  {
    WARNING("The size ({}) of the joint vector for {} is unexpected...", joint_states.size(),
            getRobotName());
  }

  const auto prev_orientation = joint_states[2];
  // hacky, last week of competition code reuse. This changes orientation, that's why I
  // saved the orientation above first.
  patchJointStateValuesToGoTo2DPose(joint_states, pose);

  const auto pose_in_rail = frame_transformer_->transformPoseToFrame(pose, long_rail_frame_id_);
  const auto aim_in_rail = frame_transformer_->transformPoseToFrame(aim, long_rail_frame_id_);

  const auto x_director_in_rail =
      aim_in_rail.position().vector() - pose_in_rail.position().vector();

  auto closest_angular_alias = [](const double next, const double prev) {
    const auto diff = std::fmod(next - prev + 2 * M_PI, 4 * M_PI) - 2 * M_PI;
    return prev + diff;
  };

  const auto necessary_orientation =
      std::atan2(-x_director_in_rail.x(), x_director_in_rail.y()) - M_PI / 2.0;

  joint_states[2] = closest_angular_alias(necessary_orientation, prev_orientation);
}

void PickAndPlaceAssemblyRobot::patchJointStateValuesToGetCloseToTargetPose(
    std::vector<double>& joint_states, const tijmath::RelativePose3& target) const
{
  patchJointStateValuesToGoTo2DPose(
      joint_states,
      findPoseClosestToTarget(target, scene_config_->getListOfSafeWaitingSpotHints()));
}

void PickAndPlaceAssemblyRobot::patchJointStateValuesToGoTo2DPose(
    std::vector<double>& joint_states, const tijmath::RelativePose3& target) const
{
  if (joint_states.size() != 9)
  {
    WARNING("The size ({}) of the joint vector for {} is unexpected...", joint_states.size(),
            getRobotName());
  }

  const auto target_in_rail = frame_transformer_->transformPoseToFrame(target, long_rail_frame_id_);

  // pay attention to the conversion from rail pose to rail values!
  joint_states[0] = target_in_rail.position().vector().y();
  joint_states[1] = target_in_rail.position().vector().x();

  const auto rotation_matrix = target_in_rail.rotation().rotationMatrix();
  // this is the direction of the x director vector
  const auto x_director_in_rail = rotation_matrix.col(0);
  joint_states[2] = std::atan2(-x_director_in_rail.x(), x_director_in_rail.y());
}

bool PickAndPlaceAssemblyRobot::testIfRobotReachesPose(
    const tijmath::RelativePose3& /*target*/) const
{
  return true;
}

tijmath::RelativePose3 PickAndPlaceAssemblyRobot::findPoseClosestToTarget(
    const tijmath::RelativePose3& target,
    const std::vector<tijmath::RelativePose3>& pose_hints) const
{
  const auto target_in_world =
      frame_transformer_->transformPoseToFrame(target, scene_config_->getWorldFrameId());

  auto shortest_distance_to_reference_sorter = [this, &reference = target_in_world](
                                                   const tijmath::RelativePose3& lhs,
                                                   const tijmath::RelativePose3& rhs) {
    const auto lhs_in_world =
        frame_transformer_->transformPoseToFrame(lhs, scene_config_->getWorldFrameId());
    const auto rhs_in_world =
        frame_transformer_->transformPoseToFrame(rhs, scene_config_->getWorldFrameId());
    auto distance_vector_left = (reference.position().vector() - lhs_in_world.position().vector());
    auto distance_vector_right = (reference.position().vector() - rhs_in_world.position().vector());
    // ignore height differences
    distance_vector_left.z() = 0.0;
    distance_vector_right.z() = 0.0;
    const auto squared_distance_left = distance_vector_left.norm();
    const auto squared_distance_right = distance_vector_right.norm();
    return squared_distance_left < squared_distance_right;
  };

  const auto closest_hint_it =
      std::min_element(pose_hints.begin(), pose_hints.end(), shortest_distance_to_reference_sorter);

  return *closest_hint_it;
}

tijmath::RelativePose3 PickAndPlaceAssemblyRobot::getCurrentRobotPose() const
{
  return tijmath::RelativePose3{ "torso_base", {}, {} };
}

tijmath::RelativePose3 PickAndPlaceAssemblyRobot::getCurrentEndEffectorPose() const
{
  return tijmath::RelativePose3{ getRobotEndEffectorLinkName(), {}, {} };
}

}  // namespace tijros
