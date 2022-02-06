/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// standard library
#include <algorithm>
#include <iostream>
#include <utility>

// tijcore
#include <tijcore/tasks/RobotTaskGroupRunner.hpp>

namespace tijcore
{
RobotTaskGroupRunner::RobotTaskGroupRunner() : garbage_collector_timer_{ [this] { timerCallback(); } }
{
  garbage_collector_timer_.start(std::chrono::milliseconds{ 100 });
}

RobotTaskGroupRunner::~RobotTaskGroupRunner()
{
  stopAllTasks();
  clearTerminatedTasks();
}

void RobotTaskGroupRunner::add(RobotTaskInterface::Ptr& task)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  tasks_in_execution_.emplace_back(std::move(task));
}

void RobotTaskGroupRunner::stopAllTasks()
{
  std::for_each(tasks_in_execution_.begin(), tasks_in_execution_.end(), [](RobotTaskAsyncRunner& task) {
    if (!task.terminated())
    {
      task.halt();
    }
  });
}

void RobotTaskGroupRunner::clearTerminatedTasks()
{
  auto ready_to_be_removed = [](const RobotTaskAsyncRunner& task) { return task.terminated(); };

  auto it = tasks_in_execution_.begin();
  while (it != tasks_in_execution_.end())
  {
    if (ready_to_be_removed(*it))
    {
      it = tasks_in_execution_.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

void RobotTaskGroupRunner::timerCallback()
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  clearTerminatedTasks();
}

}  // namespace tijcore
