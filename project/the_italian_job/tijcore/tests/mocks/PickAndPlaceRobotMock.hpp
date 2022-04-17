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
#include <tijcore/abstractions/PickAndPlaceRobotInterface.hpp>

namespace tijcore
{
class PickAndPlaceRobotMock : public PickAndPlaceRobotInterface
{
public:
  using Ptr = std::unique_ptr<PickAndPlaceRobotMock>;

  MOCK_CONST_METHOD0(getArmInRestingPose, bool());

  MOCK_CONST_METHOD1(getInSafePoseNearTarget, bool(const tijmath::RelativePose3& target));

  MOCK_CONST_METHOD2(contactPartFromAboveAndGrasp, bool(const tijmath::RelativePose3& target,
                                                        const tijcore::PartTypeId& part_type_id));

  MOCK_CONST_METHOD2(placePartFromAbove, bool(const tijmath::RelativePose3& target,
                                              const tijcore::PartTypeId& part_type_id));

  MOCK_CONST_METHOD0(enabled, bool());

  MOCK_CONST_METHOD0(name, std::string());

  MOCK_CONST_METHOD0(getRobotPlanningGroup, std::string());

  MOCK_CONST_METHOD0(gripperHasPartAttached, bool());

  MOCK_CONST_METHOD1(setSuctionGripper, void(const bool state));

  MOCK_CONST_METHOD1(patchJointStateValuesForArmInRestingPose, void(std::vector<double>&));

  MOCK_CONST_METHOD2(patchJointStateValuesToGetCloseToTargetPose,
                     void(std::vector<double>& joint_states, const tijmath::RelativePose3& target));

  MOCK_CONST_METHOD2(patchJointStateValuesGraspingHingPoseNearTarget,
                     void(std::vector<double>& joint_states, const tijmath::RelativePose3& target));

  MOCK_CONST_METHOD0(abortCurrentAction, void());

  MOCK_CONST_METHOD0(turnOnGripper, bool());

  MOCK_CONST_METHOD0(turnOffGripper, bool());
};

}  // namespace tijcore
