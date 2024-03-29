/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>
#include <set>
#include <string>
#include <vector>

// gtest
#include "gmock/gmock.h"

// tijcore
#include <tijcore/abstractions/PickAndPlaceRobotMovementsInterface.hpp>

namespace tijcore
{
class PickAndPlaceRobotMovementsMock : public PickAndPlaceRobotMovementsInterface
{
public:
  using Ptr = std::unique_ptr<PickAndPlaceRobotMovementsMock>;

  MOCK_CONST_METHOD0(getRobotArmInRestingPose, bool());

  MOCK_CONST_METHOD1(getRobotInSafePoseNearTarget, bool(const tijmath::RelativePose3& target));

  MOCK_CONST_METHOD1(getGripperIn3DPoseJoinSpace, bool(const tijmath::RelativePose3& target));

  MOCK_CONST_METHOD1(contactPartFromAboveAndGrasp, bool(const tijmath::RelativePose3& target));

  MOCK_CONST_METHOD2(getGripperIn3DPoseCartesianSpace,
                     bool(const tijmath::RelativePose3& target, const double dynamic_factor));

  MOCK_CONST_METHOD2(twistPartInPlace,
                     bool(tijmath::RelativePose3& target, const double offset_to_top));

  MOCK_CONST_METHOD1(getRobotTo2DPose, bool(const tijmath::RelativePose3& target));

  MOCK_CONST_METHOD0(getRobotHealthState, bool());

  MOCK_CONST_METHOD0(getRobotName, std::string());

  MOCK_CONST_METHOD0(getRobotGripperAttachementState, bool());

  MOCK_CONST_METHOD1(setRobotGripperState, void(const bool state));

  MOCK_CONST_METHOD0(abortCurrentAction, void());

  MOCK_CONST_METHOD0(getRobotGripperOn, bool());

  MOCK_CONST_METHOD0(getRobotGripperOff, bool());

  MOCK_CONST_METHOD1(setRobotGripperToolType, bool(const tijcore::GripperTypeId new_type));

  MOCK_CONST_METHOD0(getRobotGripperToolType, tijcore::GripperTypeId());

  MOCK_CONST_METHOD1(testIfRobotReachesPose, bool(const tijmath::RelativePose3& target));

  MOCK_METHOD2(setRobotGripperPayloadEnvelope, bool(const PayloadEnvelope& payload_envelope,
                                                    const tijmath::Isometry& relative_pose));

  MOCK_METHOD0(removeRobotGripperPayloadEnvelope, bool());

  MOCK_CONST_METHOD2(calculateVerticalLandingPose,
                     tijmath::RelativePose3(const tijmath::RelativePose3& target,
                                            const double offset_to_top));

  MOCK_CONST_METHOD2(calculateVerticalGripEndEffectorPose,
                     tijmath::RelativePose3(const tijmath::RelativePose3& target,
                                            const double offset_to_top));

  MOCK_CONST_METHOD2(calculateVerticalDropPose,
                     tijmath::RelativePose3(const tijmath::RelativePose3& target,
                                            const double offset_to_top));

  MOCK_CONST_METHOD2(calculatePayloadIntoEndEffectorTransform,
                     tijmath::Isometry(const tijmath::RelativePose3& end_effector_pose,
                                       const tijmath::RelativePose3& payload_pose));

  MOCK_CONST_METHOD2(rotateRobotToFaceTarget, bool(const tijmath::RelativePose3& target,
                                                   const tijmath::RelativePose3& aim_target));

  MOCK_CONST_METHOD0(getCurrentRobotPose, tijmath::RelativePose3());

  MOCK_CONST_METHOD0(getCurrentEndEffectorPose, tijmath::RelativePose3());
};

}  // namespace tijcore
