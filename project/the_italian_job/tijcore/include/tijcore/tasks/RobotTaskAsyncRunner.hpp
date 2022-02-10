/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <atomic>
#include <memory>
#include <thread>
#include <utility>

// tijcore
#include <tijcore/tasks/RobotTaskInterface.hpp>
#include <tijlogger/logger.hpp>

namespace tijcore
{
class RobotTaskAsyncRunner
{
public:
  using Ptr = std::unique_ptr<RobotTaskAsyncRunner>;
  using SharedPtr = std::shared_ptr<RobotTaskAsyncRunner>;

  explicit RobotTaskAsyncRunner(RobotTaskInterface::Ptr&& task) : task_{ std::move(task) }
  {
    if (!task_)
    {
      throw std::invalid_argument{ "Can't create a RobotTaskAsyncRunner from a null task ptr" };
    }
    background_thread_ = std::make_unique<std::thread>([this]() {
      WARNING("Task initiated");
      task_->run();
      terminated_ = true;
      WARNING("Task terminated");
    });
  }

  ~RobotTaskAsyncRunner()
  {
    if (background_thread_)
    {
      joinTask();
    }
  }

  void halt()
  {
    if (background_thread_)
    {
      task_->halt();
      joinTask();
    }
  }

  bool terminated() const
  {
    return terminated_;
  }

private:
  RobotTaskInterface::Ptr task_;
  std::unique_ptr<std::thread> background_thread_;
  std::atomic_bool terminated_{ false };

  void joinTask()
  {
    background_thread_->join();
    background_thread_.reset();
    task_.reset();
  }
};

}  // namespace tijcore
