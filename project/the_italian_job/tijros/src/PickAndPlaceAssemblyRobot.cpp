/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library
#include <chrono>
#include <cmath>

// ros
#include <ros/ros.h>

// tijcore
#include <logger/logger.hpp>
#include <tijcore/utils/angles.hpp>
#include <tijros/PickAndPlaceAssemblyRobot.hpp>

namespace tijros {

namespace {
// TODO(glpuga) move this into the scene configuration
constexpr char long_rail_frame_id_[] = "long_rail_1";
} // namespace

using tijcore::utils::angles::degreesToRadians;

PickAndPlaceAssemblyRobot::PickAndPlaceAssemblyRobot(
    const tijcore::Toolbox::SharedPtr &toolbox)
    : PickAndPlaceRobotCommonImpl(toolbox),
      frame_transformer_{toolbox->getFrameTransformer()},
      scene_config_{toolbox->getSceneConfigReader()},
      robot_actuator_{toolbox->getRobotActuator()} {}

bool PickAndPlaceAssemblyRobot::enabled() const {
  auto health_status = robot_actuator_->getRobotHealthStatus();
  return health_status.assembly_robot_enabled;
}

std::set<tijcore::WorkRegionId>
PickAndPlaceAssemblyRobot::supportedRegions() const {
  auto health_status = robot_actuator_->getRobotHealthStatus();
  // gantry is much slower to get in and out the from kitting zone. If
  // kitting is enabled, let the kitting robot move stuff within the near
  // kitting region
  if (health_status.kitting_robot_enabled) {
    return {tijcore::WorkRegionId::assembly,
            tijcore::WorkRegionId::kitting_agvs,
            tijcore::WorkRegionId::kitting_far_bins};
  }
  return {tijcore::WorkRegionId::assembly, tijcore::WorkRegionId::kitting_agvs,
          tijcore::WorkRegionId::kitting_near_bins,
          tijcore::WorkRegionId::kitting_far_bins};
}

std::string PickAndPlaceAssemblyRobot::name() const { return "gantry"; }

std::string PickAndPlaceAssemblyRobot::getRobotPlanningGroup() const {
  return "gantry_full";
}

bool PickAndPlaceAssemblyRobot::gripperHasPartAttached() const {
  auto gripper_state = robot_actuator_->getGantryGripperState();
  return gripper_state.attached;
}

void PickAndPlaceAssemblyRobot::setSuctionGripper(const bool state) const {
  robot_actuator_->setGantryGripperSuction(state);
}

void PickAndPlaceAssemblyRobot::patchJointStateValuesForRestingPose(
    std::vector<double> &joint_states) const {
  if (joint_states.size() != 9) {
    WARNING("The size ({}) of the joint vector for {} is unexpected...",
            joint_states.size(), name());
  }
  // Note that rail coordinates are relative to the rail, not to world
  joint_states[2] = 0.0;
  joint_states[3] = degreesToRadians(180);
  joint_states[4] = degreesToRadians(-90);
  joint_states[5] = degreesToRadians(-90);
  joint_states[6] = degreesToRadians(0);
  joint_states[7] = degreesToRadians(90);
  joint_states[8] = degreesToRadians(0);

  // reuse current_pose_estimation to get us to the closest safe pose from where
  // we are now. This requires recovering the coordinates in the rail frame from
  // the current rail values.
  tijcore::RelativePose3 current_pose_estimation{
      long_rail_frame_id_,
      tijcore::Position::fromVector(joint_states[1], joint_states[0], 0.0),
      {}};
  patchJointStateValuesToGetCloseToTarget(joint_states,
                                          current_pose_estimation);
}

void PickAndPlaceAssemblyRobot::patchJointStateValuesToGetCloseToTarget(
    std::vector<double> &joint_states,
    const tijcore::RelativePose3 &target) const {
  patchJointStateValuesToGetNearPose(
      joint_states, target, scene_config_->getListOfSafeWaitingSpotHints());
}

void PickAndPlaceAssemblyRobot::patchJointStateValuesGraspingHingPoseNearTarget(
    std::vector<double> &joint_states,
    const tijcore::RelativePose3 &target) const {
  patchJointStateValuesToGetNearPose(
      joint_states, target, scene_config_->getListOfGantryPlanningHints());
}

void PickAndPlaceAssemblyRobot::patchJointStateValuesToGetNearPose(
    std::vector<double> &joint_states, const tijcore::RelativePose3 &target,
    const std::vector<tijcore::RelativePose3> &pose_hints) const {
  if (joint_states.size() != 9) {
    WARNING("The size ({}) of the joint vector for {} is unexpected...",
            joint_states.size(), name());
  }
  const auto target_in_world = frame_transformer_->transformPoseToFrame(
      target, scene_config_->getWorldFrameId());

  auto shortest_distance_to_reference_sorter =
      [this, &reference = target_in_world](const tijcore::RelativePose3 &lhs,
                                           const tijcore::RelativePose3 &rhs) {
        const auto lhs_in_world = frame_transformer_->transformPoseToFrame(
            lhs, scene_config_->getWorldFrameId());
        const auto rhs_in_world = frame_transformer_->transformPoseToFrame(
            rhs, scene_config_->getWorldFrameId());
        auto distance_vector_left =
            (reference.position().vector() - lhs_in_world.position().vector());
        auto distance_vector_right =
            (reference.position().vector() - rhs_in_world.position().vector());
        // ignore height differences
        distance_vector_left.z() = 0.0;
        distance_vector_right.z() = 0.0;
        const auto squared_distance_left = distance_vector_left.norm();
        const auto squared_distance_right = distance_vector_right.norm();
        return squared_distance_left < squared_distance_right;
      };

  const auto closest_hint =
      std::min_element(pose_hints.begin(), pose_hints.end(),
                       shortest_distance_to_reference_sorter);
  const auto target_in_rail = frame_transformer_->transformPoseToFrame(
      *closest_hint, long_rail_frame_id_);

  // pay attention to the conversion from rail pose to rail values!
  joint_states[0] = target_in_rail.position().vector().y();
  joint_states[1] = target_in_rail.position().vector().x();

  const auto rotation_matrix = target_in_rail.rotation().rotationMatrix();
  // this is the direction of the x director vector
  const auto x_director_in_rail = rotation_matrix.col(0);
  joint_states[2] = std::atan2(-x_director_in_rail.x(), x_director_in_rail.y());

  // TODO(glpuga) test code, remove this if this does not pan out
  if (closest_hint->position().vector().z() > 5) {
    joint_states[3] = degreesToRadians(180);
    joint_states[4] = degreesToRadians(00);
    joint_states[5] = degreesToRadians(-90);
    joint_states[6] = degreesToRadians(-90);
    joint_states[7] = degreesToRadians(90);
    joint_states[8] = degreesToRadians(0);
  }
}

void PickAndPlaceAssemblyRobot::patchJointStateValuesForAlignedZeroWrist(
    std::vector<double> &joint_states) const {
  if (joint_states.size() != 9) {
    WARNING("The size ({}) of the joint vector for {} is unexpected...",
            joint_states.size(), name());
  }
  joint_states[8] = degreesToRadians(0);
}

} // namespace tijros
