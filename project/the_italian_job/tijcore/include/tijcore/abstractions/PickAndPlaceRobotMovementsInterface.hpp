/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>
#include <set>
#include <string>
#include <vector>

// tijcore
#include <tijcore/datatypes/GripperTypeId.hpp>
#include <tijcore/datatypes/PartTypeId.hpp>
#include <tijcore/utils/PayloadEnvelope.hpp>
#include <tijmath/Isometry.hpp>
#include <tijmath/RelativePose3.hpp>

namespace tijcore
{
class PickAndPlaceRobotMovementsInterface
{
public:
  using Ptr = std::unique_ptr<PickAndPlaceRobotMovementsInterface>;
  using SharedPtr = std::shared_ptr<PickAndPlaceRobotMovementsInterface>;

  virtual ~PickAndPlaceRobotMovementsInterface() = default;

  virtual bool getRobotArmInRestingPose() const = 0;

  virtual bool getRobotInSafePoseNearTarget(const tijmath::RelativePose3& target) const = 0;

  virtual bool getGripperIn3DPoseJoinSpace(const tijmath::RelativePose3& target) const = 0;

  virtual bool
  contactPartFromAboveAndGrasp(const tijmath::RelativePose3& target_end_effector_pose) const = 0;

  virtual bool getGripperIn3DPoseCartesianSpace(const tijmath::RelativePose3& target,
                                                const double dynamic_factor) const = 0;

  virtual bool twistPartInPlace(tijmath::RelativePose3& target,
                                const double offset_to_top) const = 0;

  virtual bool getRobotTo2DPose(const tijmath::RelativePose3& target) const = 0;

  virtual bool rotateRobotToFaceTarget(const tijmath::RelativePose3& target,
                                       const tijmath::RelativePose3& aim_target) const = 0;

  virtual bool getRobotGripperOn() const = 0;

  virtual bool getRobotGripperOff() const = 0;

  virtual bool getRobotGripperAttachementState() const = 0;

  virtual bool getRobotHealthState() const = 0;

  virtual std::string getRobotName() const = 0;

  virtual void abortCurrentAction() const = 0;

  virtual bool setRobotGripperToolType(const tijcore::GripperTypeId new_type) const = 0;

  virtual tijcore::GripperTypeId getRobotGripperToolType() const = 0;

  virtual bool testIfRobotReachesPose(const tijmath::RelativePose3& target) const = 0;

  virtual void setRobotGripperState(const bool state) const = 0;

  virtual bool
  setRobotGripperPayloadEnvelope(const PayloadEnvelope& payload_envelope,
                                 const tijmath::Isometry& payload_into_end_effector_transform) = 0;

  virtual bool removeRobotGripperPayloadEnvelope() = 0;

  virtual tijmath::RelativePose3 calculateVerticalLandingPose(const tijmath::RelativePose3& target,
                                                              const double offset_to_top) const = 0;

  virtual tijmath::RelativePose3 calculateVerticalGripEndEffectorPose(
      const tijmath::RelativePose3& target, const double offset_to_top) const = 0;

  virtual tijmath::RelativePose3 calculateVerticalDropPose(const tijmath::RelativePose3& target,
                                                           const double offset_to_top) const = 0;

  virtual tijmath::Isometry
  calculatePayloadIntoEndEffectorTransform(const tijmath::RelativePose3& end_effector_pose,
                                           const tijmath::RelativePose3& payload_pose) const = 0;

  virtual tijmath::RelativePose3 getCurrentRobotPose() const = 0;

  virtual tijmath::RelativePose3 getCurrentEndEffectorPose() const = 0;
};

}  // namespace tijcore
