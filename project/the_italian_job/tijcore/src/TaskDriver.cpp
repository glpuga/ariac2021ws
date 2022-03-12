/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// standard library
#include <memory>
#include <mutex>
#include <utility>

// tijcore
#include <tijcore/tasking/TaskDriver.hpp>
#include <tijlogger/logger.hpp>

namespace tijcore
{
using tijutils::Timer;

TaskDriver::TaskDriver(TaskMasterInterface::Ptr&& task_master,
                       RobotTaskGroupRunnerInterface::Ptr&& task_group_runner,
                       const ResourceManagerInterface::SharedPtr& resource_manager,
                       const ModelPerceptionInterface::SharedPtr& perception_input,
                       const Toolbox::SharedPtr& toolbox,
                       const std::chrono::milliseconds perception_interval,
                       const std::chrono::milliseconds processing_interval)
  : processing_interval_{ processing_interval }
  , task_master_{ std::move(task_master) }
  , task_group_runner_{ std::move(task_group_runner) }
  , resource_manager_{ resource_manager }
  , perception_input_{ perception_input }
  , toolbox_{ toolbox }
{
  processing_driver_ = std::make_unique<Timer>([this]() {
    perceptionCallback();
    processingCallback();
  });

  processing_driver_->start(processing_interval_);
}

void TaskDriver::perceptionCallback()
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  auto observed_models = perception_input_->getObservedModels();
  INFO("Updating sensor information with {} observed models", observed_models.size());
  resource_manager_->updateSensorData(observed_models);
  resource_manager_->logKnownLoci();
}

void TaskDriver::processingCallback()
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  auto orders = toolbox_->getProcessManager()->getOrders();
  for (const auto& order : orders)
  {
    INFO("New order received: {} ", order.order_id.codedString());
    task_master_->registerOrder(order);
  }

  auto robot_actions_to_execute = task_master_->run();

  if (!robot_actions_to_execute.empty())
  {
    INFO("Got {} new tasks, initiating execution", robot_actions_to_execute.size());
    for (auto& action : robot_actions_to_execute)
    {
      task_group_runner_->add(action);
    }
  }
}

}  // namespace tijcore
