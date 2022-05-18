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
    do
    {
      btretval = behavior_tree_->run();

      if (btretval != BTExecutionResult::SUCCESS)
      {
        std::this_thread::sleep_for(tick_interval_);
      }
      else
      {
        break;
      }
    } while (!halting_);

    return btretval == BTExecutionResult::SUCCESS ? RobotTaskOutcome::TASK_SUCCESS :
                                                    RobotTaskOutcome::TASK_FAILURE;
  }

  void halt() override
  {
    halting_ = true;
  }

private:
  std::chrono::milliseconds tick_interval_{ 100 };
  behavior_tree_extras::BehaviorTreeManagerInterface::Ptr behavior_tree_;

  std::atomic_bool halting_{ false };
};

}  // namespace tijcore
