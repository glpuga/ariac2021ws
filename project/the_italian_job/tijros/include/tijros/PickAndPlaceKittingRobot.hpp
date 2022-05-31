/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <string>
#include <vector>

// tijcore
#include <tijcore/abstractions/PickAndPlaceRobotSpecificInterface.hpp>
#include <tijcore/coremodels/Toolbox.hpp>
#include <tijcore/datatypes/GripperTypeId.hpp>

namespace tijros
{
class PickAndPlaceKittingRobot : public tijcore::PickAndPlaceRobotSpecificInterface
{
public:
  explicit PickAndPlaceKittingRobot(const tijcore::Toolbox::SharedPtr& toolbox);

  std::string getRobotName() const override;

  bool getRobotHealthState() const override;

  bool getRobotGripperAttachementState() const override;

  bool testIfRobotReachesPose(const tijmath::RelativePose3& target) const override;

  std::string getRobotPlanningGroup() const override;

  std::string getRobotEndEffectorLinkName() const override;

  void setRobotGripperState(const bool state) const override;

  bool setGripperToolTypeImpl(const tijcore::GripperTypeId new_type) const override;

  tijcore::GripperTypeId getGripperToolTypeImpl() const override;

  void patchJointStateValuesForArmInRestingPose(std::vector<double>&) const override;

  void patchJointStateValuesToGetCloseToTargetPose(
      std::vector<double>& joint_states, const tijmath::RelativePose3& target) const override;

  void patchJointStateValuesToGoTo2DPose(std::vector<double>& joint_states,
                                         const tijmath::RelativePose3& target) const override;

  void patchJointStateValuesToFaceTarget(std::vector<double>& joint_states,
                                         const tijmath::RelativePose3& pose,
                                         const tijmath::RelativePose3& aim) const override;

  void patchJointStateValuesForAlignedZeroWrist(std::vector<double>& joint_states) const override;

  tijmath::RelativePose3 getCurrentRobotPose() const override;

  tijmath::RelativePose3 getCurrentEndEffectorPose() const override;

private:
  tijcore::FrameTransformerInterface::SharedPtr frame_transformer_;
  tijcore::RobotActuatorsInterface::SharedPtr robot_actuator_;
};

}  // namespace tijros
