/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <chrono>
#include <memory>
#include <mutex>

// tijcore
#include <tijcore/perception/ModelPerceptionInterface.hpp>
#include <tijcore/perception/ResourceManagerInterface.hpp>
#include <tijcore/perception/TaskMasterInterface.hpp>
#include <tijcore/perception/Toolbox.hpp>
#include <tijcore/tasks/RobotTaskGroupRunnerInterface.hpp>
#include <tijcore/utils/Timer.hpp>

using namespace std::literals;

namespace tijcore
{
class TaskDriver
{
public:
  using Ptr = std::unique_ptr<TaskDriver>;
  using SharedPtr = std::shared_ptr<TaskDriver>;

  TaskDriver(TaskMasterInterface::Ptr&& task_master, RobotTaskGroupRunnerInterface::Ptr&& task_group_runner,
             const ResourceManagerInterface::SharedPtr& resource_manager,
             const ModelPerceptionInterface::SharedPtr& perception_input, const Toolbox::SharedPtr& toolbox,
             const std::chrono::milliseconds perception_interval = 1s,
             const std::chrono::milliseconds processing_interval = 1s);

private:
  const std::chrono::milliseconds processing_interval_{ 1000 };

  TaskMasterInterface::Ptr task_master_;
  RobotTaskGroupRunnerInterface::Ptr task_group_runner_;
  ResourceManagerInterface::SharedPtr resource_manager_;
  ModelPerceptionInterface::SharedPtr perception_input_;
  Toolbox::SharedPtr toolbox_;

  std::mutex mutex_;

  std::unique_ptr<utils::Timer> processing_driver_;

  void perceptionCallback();

  void processingCallback();
};

}  // namespace tijcore
