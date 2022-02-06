/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>

// tijcore
#include <tijcore/tasks/RobotTaskInterface.hpp>

namespace tijcore
{
class RobotTaskGroupRunnerInterface
{
public:
  using Ptr = std::unique_ptr<RobotTaskGroupRunnerInterface>;
  using SharedPtr = std::shared_ptr<RobotTaskGroupRunnerInterface>;

  virtual ~RobotTaskGroupRunnerInterface() = default;

  // the parameter should be and rvalue-ref, but this is a compromise solution
  // to be able to mock the class
  virtual void add(RobotTaskInterface::Ptr& task) = 0;
};

}  // namespace tijcore
