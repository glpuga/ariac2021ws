/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>
#include <vector>

// gtest
#include "gmock/gmock.h"

// tijcore
#include <tijcore/abstractions/TaskMasterInterface.hpp>

namespace tijcore
{
class TaskMasterMock : public TaskMasterInterface
{
public:
  using Ptr = std::unique_ptr<TaskMasterMock>;

  MOCK_METHOD1(registerOrder, void(const Order& order));

  MOCK_METHOD0(run, std::vector<RobotTaskInterface::Ptr>());
};

}  // namespace tijcore
