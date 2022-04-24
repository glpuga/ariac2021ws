/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <set>
#include <string>
#include <vector>

// tijcore
#include <tijcore/coremodels/Toolbox.hpp>
#include <tijros/PickAndPlaceRobotCommonImpl.hpp>

namespace tijros
{
class PickAndPlaceKittingRobot : public PickAndPlaceRobotCommonImpl
{
public:
  explicit PickAndPlaceKittingRobot(const tijcore::Toolbox::SharedPtr& toolbox);

  bool enabled() const override;

  std::string name() const override;

  bool gripperHasPartAttached() const override;

  bool canReach(const tijmath::RelativePose3& target) const override;

private:
  tijcore::FrameTransformerInterface::SharedPtr frame_transformer_;
  tijcore::RobotActuatorsInterface::SharedPtr robot_actuator_;

  std::string getRobotPlanningGroup() const override;

  void setSuctionGripper(const bool state) const override;

  void patchJointStateValuesForArmInRestingPose(std::vector<double>&) const override;

  void patchJointStateValuesToGetCloseToTargetPose(
      std::vector<double>& joint_states, const tijmath::RelativePose3& target) const override;

  void patchJointStateValuesGraspingHingPoseNearTarget(
      std::vector<double>& joint_states, const tijmath::RelativePose3& target) const override;

  bool setGripperToolTypeImpl(const tijcore::GripperTypeId new_type) const override;

  tijcore::GripperTypeId getGripperToolTypeImpl() const override;
};

}  // namespace tijros
