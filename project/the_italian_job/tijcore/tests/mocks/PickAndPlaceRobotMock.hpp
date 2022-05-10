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
class PickAndPlaceRobotMock : public PickAndPlaceRobotMovementsInterface
{
public:
  using Ptr = std::unique_ptr<PickAndPlaceRobotMock>;

  MOCK_CONST_METHOD0(getRobotArmInRestingPose, bool());

  MOCK_CONST_METHOD1(getRobotInSafePoseNearTarget, bool(const tijmath::RelativePose3& target));

  MOCK_CONST_METHOD1(getGripperInLandingSpot, bool(const tijmath::RelativePose3& target));

  MOCK_CONST_METHOD2(contactPartFromAboveAndGrasp, bool(const tijmath::RelativePose3& target,
                                                        const tijcore::PartTypeId& part_type_id));

  MOCK_CONST_METHOD2(placePartFromAbove, bool(const tijmath::RelativePose3& target,
                                              const tijcore::PartTypeId& part_type_id));

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
};

}  // namespace tijcore
