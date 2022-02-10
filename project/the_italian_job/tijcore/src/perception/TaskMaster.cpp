/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// standard library
#include <algorithm>
#include <memory>
#include <mutex>
#include <set>
#include <utility>
#include <vector>

// tijcore
#include <tijcore/perception/TaskMaster.hpp>
#include <tijlogger/logger.hpp>

namespace tijcore
{
TaskMaster::TaskMaster(const ResourceManagerInterface::SharedPtr& resource_manager,
                       const RobotTaskFactoryInterface::SharedPtr& robot_task_factory,
                       const Toolbox::SharedPtr& toolbox, OrderProcessingStrategyInterface::Ptr&& order_strategy)
  : resource_manager_{ resource_manager }, robot_task_factory_{ robot_task_factory }, toolbox_{ toolbox }
{
  order_strategy_ = std::move(order_strategy);
  if (!order_strategy_)
  {
    order_strategy_ = std::make_unique<OrderProcessingStrategy>(resource_manager, robot_task_factory, toolbox_);
  }
  else
  {
  }
}

void TaskMaster::registerOrder(const Order& order)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  auto key = order.order_id.id();

  if (orders_.count(key))
  {
    if (!order.order_id.isUpdate())
    {
      WARNING(
          "A new order was received with the same id as a previous one "
          "({}), but its not marked as an update",
          order.order_id.codedString());
      return;
    }
    else
    {
      // replace the current order
      WARNING("A new order was received updating one with the same id: {}", order.order_id.codedString());
      orders_.at(key) = order;
    }
  }

  // the order is completely new
  orders_.emplace(std::make_pair(key, order));
}

std::vector<RobotTaskInterface::Ptr> TaskMaster::run()
{
  std::lock_guard<std::mutex> lock{ mutex_ };

  auto ger_order_keys_by_priority = [this]() {
    std::vector<OrderKey> keys;
    for (const auto& [key, data] : orders_)
    {
      (void)data;  // avoids unused variable warning
      keys.push_back(key);
    }
    auto priority_sort = [](const OrderKey& a, const OrderKey& b) -> bool {
      // should return true if for higher order numbers (order_1 before
      // order_0)
      return a > b;
    };
    std::sort(keys.begin(), keys.end(), priority_sort);
    return keys;
  };

  auto priority_sorted_keys = ger_order_keys_by_priority();

  std::set<AgvId> agsv_in_use;
  std::set<StationId> stations_in_use;

  for (const auto& key : priority_sorted_keys)
  {
    auto& order = orders_.at(key);
    order_strategy_->getAgvsAndStationsInUse(order, agsv_in_use, stations_in_use);
  }

  std::vector<RobotTaskInterface::Ptr> tasks;

  for (const auto& key : priority_sorted_keys)
  {
    auto& order = orders_.at(key);
    order_strategy_->disambiguateOrderEndpoints(order, agsv_in_use, stations_in_use);
    auto order_tasks = order_strategy_->processOrder(order, agsv_in_use, stations_in_use);

    std::move(order_tasks.begin(), order_tasks.end(), std::back_inserter(tasks));

    // if there are no more shipments from this order, remove it
    if (order.kitting_shipments.empty() && order.assembly_shipments.empty())
    {
      INFO("Completed order {}", order.order_id.codedString());
      orders_.erase(key);
    }
  }

  return tasks;
}

}  // namespace tijcore
