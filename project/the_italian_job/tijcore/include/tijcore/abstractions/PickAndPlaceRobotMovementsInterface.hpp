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

  virtual bool contactPartFromAboveAndGrasp(const tijmath::RelativePose3& target,
                                            const double offset_to_top) const = 0;

  virtual bool getGripperIn3DPoseCartesianSpace(const tijmath::RelativePose3& target) const = 0;

  virtual bool getRobotTo2DPose(const tijmath::RelativePose3& target) const = 0;

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

  virtual tijmath::RelativePose3
  calculateVerticalLandingPose(const tijmath::RelativePose3& target) const = 0;

  virtual tijmath::RelativePose3 calculateVerticalDropPose(const tijmath::RelativePose3& target,
                                                           const double offset_to_top) const = 0;
};

}  // namespace tijcore
