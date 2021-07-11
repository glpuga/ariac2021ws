/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// gtest
#include "gmock/gmock.h"

// tijcore
#include <tijcore/tasks/RobotTaskInterface.hpp>

namespace tijcore {

class RobotTaskMock : public RobotTaskInterface {
public:
  using Ptr = std::unique_ptr<RobotTaskMock>;

  MOCK_METHOD0(run, RobotTaskOutcome());
  MOCK_METHOD0(halt, void());
};

} // namespace tijcore
