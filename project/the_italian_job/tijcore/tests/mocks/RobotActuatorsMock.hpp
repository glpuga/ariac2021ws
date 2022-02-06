/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>

// gtest
#include "gmock/gmock.h"

// tijcore
#include <tijcore/competition/RobotActuatorsInterface.hpp>

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

  MOCK_CONST_METHOD0(getKittingGripperState, VacuumGripperState());

  MOCK_CONST_METHOD1(setKittingGripperSuction, bool(const bool enable));

  MOCK_CONST_METHOD1(setGantryTrayLockState, bool(const bool lock_state));

  MOCK_CONST_METHOD0(getRobotHealthStatus, RobotHealthStatus());
};

}  // namespace tijcore
