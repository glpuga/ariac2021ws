/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <algorithm>
#include <set>
#include <string>
#include <vector>

// tijcore
#include <tijcore/coremodels/Toolbox.hpp>
#include <tijros/PickAndPlaceRobotCommonImpl.hpp>

namespace tijros
{
class PickAndPlaceAssemblyRobot : public PickAndPlaceRobotCommonImpl
{
public:
  explicit PickAndPlaceAssemblyRobot(const tijcore::Toolbox::SharedPtr& toolbox);

  bool enabled() const override;

  std::set<tijcore::WorkRegionId> supportedRegions() const override;

  std::string name() const override;

  bool gripperHasPartAttached() const override;

private:
  tijcore::FrameTransformerInterface::SharedPtr frame_transformer_;
  tijcore::SceneConfigReaderInterface::SharedPtr scene_config_;
  tijcore::RobotActuatorsInterface::SharedPtr robot_actuator_;

  std::string getRobotPlanningGroup() const override;

  void setSuctionGripper(const bool state) const override;

  void patchJointStateValuesForRestingPose(std::vector<double>&) const override;

  void patchJointStateValuesToGetCloseToTarget(std::vector<double>& joint_states,
                                               const tijmath::RelativePose3& target) const override;

  void patchJointStateValuesGraspingHingPoseNearTarget(std::vector<double>& joint_states,
                                                       const tijmath::RelativePose3& target) const override;

  void patchJointStateValuesToGetNearPose(std::vector<double>& joint_states, const tijmath::RelativePose3& target,
                                          const std::vector<tijmath::RelativePose3>& pose_hints) const;

  void patchJointStateValuesForAlignedZeroWrist(std::vector<double>& joint_states) const override;
};

}  // namespace tijros
