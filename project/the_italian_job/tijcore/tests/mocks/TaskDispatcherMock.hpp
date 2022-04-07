/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>
#include <vector>

// gtest
#include "gmock/gmock.h"

// tijcore
#include <tijcore/abstractions/TaskDispatcherInterface.hpp>

namespace tijcore
{
class TaskDispatcherMock : public TaskDispatcherInterface
{
public:
  using Ptr = std::unique_ptr<TaskDispatcherMock>;

  MOCK_METHOD1(registerOrder, void(const Order& order));

  MOCK_METHOD0(run, std::vector<RobotTaskInterface::Ptr>());
};

}  // namespace tijcore
