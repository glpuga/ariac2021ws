/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library
#include <chrono>
#include <set>
#include <string>
#include <vector>

// ros
#include <ros/ros.h>

// tijcore
#include <tijcore/logger/logger.hpp>
#include <tijcore/utils/angles.hpp>
#include <tijros/PickAndPlaceKittingRobot.hpp>

namespace tijros
{
using tijcore::utils::angles::degreesToRadians;

PickAndPlaceKittingRobot::PickAndPlaceKittingRobot(const tijcore::Toolbox::SharedPtr& toolbox)
  : PickAndPlaceRobotCommonImpl(toolbox)
  , frame_transformer_{ toolbox->getFrameTransformer() }
  , robot_actuator_{ toolbox->getRobotActuator() }
{
}

bool PickAndPlaceKittingRobot::enabled() const
{
  auto health_status = robot_actuator_->getRobotHealthStatus();
  return health_status.kitting_robot_enabled;
}

std::set<tijcore::WorkRegionId> PickAndPlaceKittingRobot::supportedRegions() const
{
  return { tijcore::WorkRegionId::kitting_agvs, tijcore::WorkRegionId::kitting_near_bins,
           tijcore::WorkRegionId::conveyor_belt };
}

std::string PickAndPlaceKittingRobot::name() const
{
  return "kitting";
}

std::string PickAndPlaceKittingRobot::getRobotPlanningGroup() const
{
  return "kitting_arm";
}

bool PickAndPlaceKittingRobot::gripperHasPartAttached() const
{
  auto gripper_state = robot_actuator_->getKittingGripperState();
  return gripper_state.attached;
}

void PickAndPlaceKittingRobot::setSuctionGripper(const bool state) const
{
  robot_actuator_->setKittingGripperSuction(state);
}

void PickAndPlaceKittingRobot::patchJointStateValuesForRestingPose(std::vector<double>& joint_states) const
{
  if (joint_states.size() != 7)
  {
    WARNING("The size ({}) of the joint vector for {} is unexpected...", joint_states.size(), name());
  }
  // these correspond to the positions of the rest position of the arm,
  // excluding the linear rail
  const auto sign = (joint_states[0] < 0) ? -1.0 : 1.0;
  joint_states[1] = 1.57 * sign;
  joint_states[2] = -1.507;
  joint_states[3] = 2.216;
  joint_states[4] = 3.92;
  joint_states[5] = -1.507;
  joint_states[6] = 0;
}

void PickAndPlaceKittingRobot::patchJointStateValuesGraspingHingPoseNearTarget(
    std::vector<double>& joint_states, const tijcore::RelativePose3& target) const
{
  if (joint_states.size() != 7)
  {
    WARNING("The size ({}) of the joint vector for {} is unexpected...", joint_states.size(), name());
  }
  const auto target_in_world = frame_transformer_->transformPoseToFrame(target, "world");
  joint_states[0] = target_in_world.position().vector().y();

  const auto part_in_belt = target_in_world.position().vector().x() > -1;
  if (part_in_belt)
  {
    joint_states[0] -= 0.25;
    joint_states[1] = degreesToRadians(-60);
  }
  else
  {
    // bias the location so that the robot is in a slight angle to grasp
    const auto sign = (joint_states[0] < 0) ? -1.0 : 1.0;
    joint_states[0] -= 1.0 * sign;
    joint_states[1] = 1.9 * sign;  // magic numbers. MoveIt's full of them.
  }
}

void PickAndPlaceKittingRobot::patchJointStateValuesToGetCloseToTarget(std::vector<double>& joint_states,
                                                                       const tijcore::RelativePose3& target) const
{
  if (joint_states.size() != 7)
  {
    WARNING("The size ({}) of the joint vector for {} is unexpected...", joint_states.size(), name());
  }

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
    const auto sign = (joint_states[0] < 0) ? -1.0 : 1.0;
    joint_states[0] -= 1.0 * sign;
    joint_states[1] = degreesToRadians(90) * sign;
  }
}

void PickAndPlaceKittingRobot::patchJointStateValuesForAlignedZeroWrist(std::vector<double>& joint_states) const
{
  if (joint_states.size() != 7)
  {
    WARNING("The size ({}) of the joint vector for {} is unexpected...", joint_states.size(), name());
  }
  joint_states[6] = degreesToRadians(0);
}

}  // namespace tijros
