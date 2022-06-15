/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>

// gtest
#include "gmock/gmock.h"

// tijcore
#include <tijcore/abstractions/RobotActuatorsInterface.hpp>

namespace tijcore
{
class RobotActuatorsMock : public RobotActuatorsInterface
{
public:
  using Ptr = std::unique_ptr<RobotActuatorsMock>;
  using SharedPtr = std::shared_ptr<RobotActuatorsMock>;

  MOCK_CONST_METHOD0(getConveyorState, ConveyorState());

  MOCK_CONST_METHOD0(getGantryGripperState, VacuumGripperState());

  MOCK_CONST_METHOD1(setGantryGripperSuction, bool(const bool enable));

  MOCK_CONST_METHOD1(setGantryGripperToolType, bool(const tijcore::GripperTypeId new_type));

  MOCK_CONST_METHOD0(getKittingGripperState, VacuumGripperState());

  MOCK_CONST_METHOD1(setKittingGripperSuction, bool(const bool enable));

  MOCK_CONST_METHOD1(setGantryTrayLockState, bool(const bool lock_state));

  MOCK_CONST_METHOD0(getRobotHealthStatus, RobotHealthStatus());

  MOCK_CONST_METHOD0(getGantryGripperToolType, tijcore::GripperTypeId());

  MOCK_METHOD0(getKittingJointDirectControlManager, tijcore::RobotJointDirectControlInterface&());

  MOCK_METHOD0(getGantryJointDirectControlManager, tijcore::RobotJointDirectControlInterface&());
};

}  // namespace tijcore
