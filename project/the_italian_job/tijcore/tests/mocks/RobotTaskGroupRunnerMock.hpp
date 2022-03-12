/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>

// gtest
#include "gmock/gmock.h"

// tijcore
#include <tijcore/abstractions/RobotTaskGroupRunnerInterface.hpp>

namespace tijcore
{
class RobotTaskGroupRunnerMock : public RobotTaskGroupRunnerInterface
{
public:
  using Ptr = std::unique_ptr<RobotTaskGroupRunnerMock>;

  MOCK_METHOD1(add, void(RobotTaskInterface::Ptr&));
};

}  // namespace tijcore
