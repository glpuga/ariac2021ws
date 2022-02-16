/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <list>
#include <mutex>

// tijcore
#include <tijcore/abstractions/RobotTaskGroupRunnerInterface.hpp>
#include <tijcore/tasking/RobotTaskAsyncRunner.hpp>
#include <tijcore/utils/Timer.hpp>

namespace tijcore
{
class RobotTaskGroupRunner : public RobotTaskGroupRunnerInterface
{
public:
  RobotTaskGroupRunner();

  ~RobotTaskGroupRunner();

  void add(RobotTaskInterface::Ptr& task) override;

private:
  std::mutex mutex_;

  std::list<RobotTaskAsyncRunner> tasks_in_execution_;
  utils::Timer garbage_collector_timer_;

  void stopAllTasks();

  void clearTerminatedTasks();

  void timerCallback();
};

}  // namespace tijcore
