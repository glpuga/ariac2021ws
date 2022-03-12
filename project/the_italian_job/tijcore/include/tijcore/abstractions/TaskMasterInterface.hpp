/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>
#include <vector>

// tijcore
#include <tijcore/abstractions/RobotTaskInterface.hpp>
#include <tijcore/datatypes/Order.hpp>

namespace tijcore
{
class TaskMasterInterface
{
public:
  using Ptr = std::unique_ptr<TaskMasterInterface>;
  using SharedPtr = std::shared_ptr<TaskMasterInterface>;

  virtual ~TaskMasterInterface() = default;

  virtual void registerOrder(const Order& order) = 0;

  virtual std::vector<RobotTaskInterface::Ptr> run() = 0;
};

}  // namespace tijcore
