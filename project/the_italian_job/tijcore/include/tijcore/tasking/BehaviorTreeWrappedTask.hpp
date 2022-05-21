/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <atomic>
#include <chrono>
#include <thread>
#include <utility>

// external
#include <behavior_tree_extras/BehaviorTreeManagerInterface.hpp>

// tijcore
#include <tijcore/abstractions/RobotTaskInterface.hpp>

namespace tijcore
{
class BehaviorTreeWrappedTask : public RobotTaskInterface
{
public:
  explicit BehaviorTreeWrappedTask(
      behavior_tree_extras::BehaviorTreeManagerInterface::Ptr behavior_tree)
    : behavior_tree_{ std::move(behavior_tree) } {};

  RobotTaskOutcome run() override
  {
    using behavior_tree_extras::BTExecutionResult;

    BTExecutionResult btretval;

    try
    {
      btretval = behavior_tree_->run();
    }
    catch (const std::exception& e)
    {
      ERROR("Exception thrown in the behavior tree tick method: {}", e.what());
      btretval = BTExecutionResult::ERROR;
    }

    if (btretval == BTExecutionResult::ERROR)
    {
      ERROR("The behavior tree tick call terminated in ERROR");
    }

    return btretval == BTExecutionResult::SUCCESS ? RobotTaskOutcome::TASK_SUCCESS :
                                                    RobotTaskOutcome::TASK_FAILURE;
  }

  void halt() override
  {
    behavior_tree_->halt();
  }

private:
  behavior_tree_extras::BehaviorTreeManagerInterface::Ptr behavior_tree_;
};

}  // namespace tijcore
