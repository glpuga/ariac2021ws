/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>
#include <string>

// roscpp
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>

// tijcore
#include <tijcore/abstractions/PickAndPlaceRobotMovementsInterface.hpp>
#include <tijcore/abstractions/PickAndPlaceRobotSpecificInterface.hpp>
#include <tijcore/coremodels/Toolbox.hpp>
#include <tijmath/RelativePose3.hpp>

namespace tijros
{
class PickAndPlaceRobotMovements : public tijcore::PickAndPlaceRobotMovementsInterface
{
public:
  PickAndPlaceRobotMovements(
      tijcore::PickAndPlaceRobotSpecificInterface::Ptr robot_specific_interface,
      const tijcore::Toolbox::SharedPtr& toolbox);

  bool getRobotArmInRestingPose() const override;

  bool getRobotInSafePoseNearTarget(const tijmath::RelativePose3& target) const override;

  bool getGripperIn3DPoseJoinSpace(const tijmath::RelativePose3& target) const override;

  bool contactPartFromAboveAndGrasp(
      const tijmath::RelativePose3& target_end_effector_pose) const override;

  bool getGripperIn3DPoseCartesianSpace(const tijmath::RelativePose3& target) const override;

  bool getRobotTo2DPose(const tijmath::RelativePose3& target) const override;

  bool rotateRobotToFaceTarget(const tijmath::RelativePose3& target) const override;

  bool getRobotGripperOn() const override;

  bool getRobotGripperOff() const override;

  void abortCurrentAction() const override;

  bool setRobotGripperToolType(const tijcore::GripperTypeId new_type) const override;

  tijcore::GripperTypeId getRobotGripperToolType() const override;

  bool getRobotGripperAttachementState() const override;

  bool getRobotHealthState() const override;

  std::string getRobotName() const override;

  bool testIfRobotReachesPose(const tijmath::RelativePose3& target) const override;

  void setRobotGripperState(const bool state) const override;

  bool setRobotGripperPayloadEnvelope(const tijcore::PayloadEnvelope& payload_envelope,
                                      const tijmath::Pose3& relative_pose) override;

  bool removeRobotGripperPayloadEnvelope() override;

  tijmath::RelativePose3 calculateVerticalLandingPose(const tijmath::RelativePose3& target,
                                                      const double offset_to_top) const override;

  tijmath::RelativePose3 calculateVerticalGripEndEffectorPose(
      const tijmath::RelativePose3& target, const double offset_to_top) const override;

  tijmath::RelativePose3 calculateVerticalDropPose(const tijmath::RelativePose3& target,
                                                   const double offset_to_top) const override;

  tijmath::Pose3
  calculateEndEffectorToPayloadTransform(const tijmath::RelativePose3& end_effector_pose,
                                         const tijmath::RelativePose3& payload_pose) const override;

private:
  tijcore::PickAndPlaceRobotSpecificInterface::Ptr robot_specific_interface_;
  tijcore::Toolbox::SharedPtr toolbox_;

  mutable std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_ptr_;
  mutable std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_ptr_;

  bool enable_payload_envelope_{ false };
  tijcore::PayloadEnvelope payload_envelope_;
  tijmath::Pose3 ee_to_payload_pose_;

  moveit::planning_interface::MoveGroupInterface* buildMoveItGroupHandle() const;

  void buildObstacleSceneFromDescription() const;

  void alignEndEffectorWithTarget(tijmath::RelativePose3& target_in_world) const;

  void useNarrowTolerances(const bool tight_mode) const;

  void createPayloadEnvelopCollisionBox() const;
};

}  // namespace tijros
