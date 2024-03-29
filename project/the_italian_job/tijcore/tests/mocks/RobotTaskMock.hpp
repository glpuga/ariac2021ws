/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>

// gtest
#include "gmock/gmock.h"

// tijcore
#include <tijcore/abstractions/RobotTaskInterface.hpp>

namespace tijcore
{
class RobotTaskMock : public RobotTaskInterface
{
public:
  using Ptr = std::unique_ptr<RobotTaskMock>;

  MOCK_METHOD0(run, RobotTaskOutcome());
  MOCK_METHOD0(halt, void());
};

}  // namespace tijcore
