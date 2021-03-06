/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <unordered_map>
#include <vector>

// tijcore
#include <tijcore/agents/Order.hpp>
#include <tijcore/perception/OrderProcessingStrategy.hpp>
#include <tijcore/perception/OrderProcessingStrategyInterface.hpp>
#include <tijcore/perception/ResourceManagerInterface.hpp>
#include <tijcore/perception/RobotTaskFactoryInterface.hpp>
#include <tijcore/perception/TaskMasterInterface.hpp>
#include <tijcore/perception/Toolbox.hpp>

namespace tijcore {

class TaskMaster : public TaskMasterInterface {
public:
  TaskMaster(const ResourceManagerInterface::SharedPtr &resource_manager,
             const RobotTaskFactoryInterface::SharedPtr &robot_task_factory,
             const Toolbox::SharedPtr &toolbox,
             OrderProcessingStrategyInterface::Ptr &&order_strategy = nullptr);

  void registerOrder(const Order &order) override;

  std::vector<RobotTaskInterface::Ptr> run() override;

private:
  using OrderKey = OrderId::OrderIdType;

  std::mutex mutex_;

  ResourceManagerInterface::SharedPtr resource_manager_;
  RobotTaskFactoryInterface::SharedPtr robot_task_factory_;
  Toolbox::SharedPtr toolbox_;

  OrderProcessingStrategyInterface::Ptr order_strategy_;

  std::unordered_map<OrderKey, Order> orders_;

  std::vector<RobotTaskInterface::Ptr> processOrder(const Order &order);
};

} // namespace tijcore
