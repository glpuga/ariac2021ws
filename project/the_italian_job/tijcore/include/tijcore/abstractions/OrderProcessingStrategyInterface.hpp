/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>
#include <set>
#include <vector>

// tijcore
#include <tijcore/abstractions/ResourceManagerInterface.hpp>
#include <tijcore/abstractions/RobotTaskInterface.hpp>
#include <tijcore/datatypes/Order.hpp>

namespace tijcore
{
class OrderProcessingStrategyInterface
{
public:
  using Ptr = std::unique_ptr<OrderProcessingStrategyInterface>;

  virtual ~OrderProcessingStrategyInterface() = default;

  virtual void getAgvsAndStationsInUse(Order& order, std::set<AgvId>& agvs_in_use,
                                       std::set<StationId>& stations_in_use) const = 0;

  virtual void disambiguateOrderEndpoints(Order& order, std::set<AgvId>& input_agvs_in_use,
                                          std::set<StationId>& stations_in_use) const = 0;

  virtual std::vector<RobotTaskInterface::Ptr>
  processOrder(Order& order, const std::set<AgvId>& agvs_in_use,
               const std::set<StationId>& stations_in_use) const = 0;
};

}  // namespace tijcore
