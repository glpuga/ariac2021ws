/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <chrono>
#include <memory>
#include <mutex>

// tijcore
#include <tijcore/abstractions/ModelPerceptionInterface.hpp>
#include <tijcore/abstractions/ResourceManagerInterface.hpp>
#include <tijcore/abstractions/RobotTaskGroupRunnerInterface.hpp>
#include <tijcore/abstractions/TaskDispatcherInterface.hpp>
#include <tijcore/coremodels/Toolbox.hpp>
#include <tijutils/Timer.hpp>

namespace tijcore
{
class TaskDriver
{
public:
  using Ptr = std::unique_ptr<TaskDriver>;
  using SharedPtr = std::shared_ptr<TaskDriver>;

  TaskDriver(TaskDispatcherInterface::Ptr&& task_master,
             RobotTaskGroupRunnerInterface::Ptr&& task_group_runner,
             const ResourceManagerInterface::SharedPtr& resource_manager,
             const ModelPerceptionInterface::SharedPtr& perception_input,
             const Toolbox::SharedPtr& toolbox,
             const std::chrono::milliseconds perception_interval = std::chrono::seconds{ 1 },
             const std::chrono::milliseconds processing_interval = std::chrono::seconds{ 1 });

private:
  const std::chrono::milliseconds processing_interval_{ 1000 };

  TaskDispatcherInterface::Ptr task_master_;
  RobotTaskGroupRunnerInterface::Ptr task_group_runner_;
  ResourceManagerInterface::SharedPtr resource_manager_;
  ModelPerceptionInterface::SharedPtr perception_input_;
  Toolbox::SharedPtr toolbox_;

  std::mutex mutex_;

  std::unique_ptr<tijutils::Timer> processing_driver_;

  void perceptionCallback();

  void processingCallback();
};

}  // namespace tijcore
