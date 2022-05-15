/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>
#include <string>
#include <vector>

// tijcore
#include <tijcore/datatypes/GripperTypeId.hpp>
#include <tijmath/RelativePose3.hpp>

namespace tijcore
{
class PickAndPlaceRobotSpecificInterface
{
public:
  using Ptr = std::unique_ptr<PickAndPlaceRobotSpecificInterface>;
  using SharedPtr = std::shared_ptr<PickAndPlaceRobotSpecificInterface>;

  virtual ~PickAndPlaceRobotSpecificInterface() = default;

  virtual std::string getRobotName() const = 0;

  virtual std::string getRobotPlanningGroup() const = 0;

  virtual std::string getRobotEndEffectorLinkName() const = 0;

  virtual bool getRobotHealthState() const = 0;

  virtual bool testIfRobotReachesPose(const tijmath::RelativePose3& target) const = 0;

  virtual void setRobotGripperState(const bool state) const = 0;

  virtual tijcore::GripperTypeId getGripperToolTypeImpl() const = 0;

  virtual void patchJointStateValuesForArmInRestingPose(std::vector<double>&) const = 0;

  virtual void patchJointStateValuesToGetCloseToTargetPose(
      std::vector<double>& joint_states, const tijmath::RelativePose3& target) const = 0;

  virtual void patchJointStateValuesToGoTo2DPose(std::vector<double>& joint_states,
                                                 const tijmath::RelativePose3& target) const = 0;

  virtual bool getRobotGripperAttachementState() const = 0;

  virtual bool setGripperToolTypeImpl(const tijcore::GripperTypeId new_type) const = 0;
};

}  // namespace tijcore
