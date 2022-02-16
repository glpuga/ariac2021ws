/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <unordered_map>
#include <vector>

// tijcore
#include <tijcore/abstractions/OrderProcessingStrategyInterface.hpp>
#include <tijcore/abstractions/ResourceManagerInterface.hpp>
#include <tijcore/abstractions/RobotTaskFactoryInterface.hpp>
#include <tijcore/abstractions/TaskMasterInterface.hpp>
#include <tijcore/coremodels/OrderProcessingStrategy.hpp>
#include <tijcore/coremodels/Toolbox.hpp>
#include <tijcore/datatypes/Order.hpp>

namespace tijcore
{
class TaskMaster : public TaskMasterInterface
{
public:
  TaskMaster(const ResourceManagerInterface::SharedPtr& resource_manager,
             const RobotTaskFactoryInterface::SharedPtr& robot_task_factory, const Toolbox::SharedPtr& toolbox,
             OrderProcessingStrategyInterface::Ptr&& order_strategy = nullptr);

  void registerOrder(const Order& order) override;

  std::vector<RobotTaskInterface::Ptr> run() override;

private:
  using OrderKey = OrderId::OrderIdType;

  std::mutex mutex_;

  ResourceManagerInterface::SharedPtr resource_manager_;
  RobotTaskFactoryInterface::SharedPtr robot_task_factory_;
  Toolbox::SharedPtr toolbox_;

  OrderProcessingStrategyInterface::Ptr order_strategy_;

  std::unordered_map<OrderKey, Order> orders_;

  std::vector<RobotTaskInterface::Ptr> processOrder(const Order& order);
};

}  // namespace tijcore
