/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>
#include <set>
#include <vector>

// gtest
#include "gmock/gmock.h"

// tijcore
#include <tijcore/abstractions/OrderProcessingStrategyInterface.hpp>

namespace tijcore
{
class OrderProcessingStrategyMock : public OrderProcessingStrategyInterface
{
public:
  using Ptr = std::unique_ptr<OrderProcessingStrategyMock>;

  MOCK_CONST_METHOD3(getAgvsAndStationsInUse, void(Order& order, std::set<AgvId>& agvs_in_use,
                                                   std::set<StationId>& stations_in_use));

  MOCK_CONST_METHOD3(disambiguateOrderEndpoints,
                     void(Order& order, std::set<AgvId>& input_agvs_in_use,
                          std::set<StationId>& stations_in_use));

  MOCK_CONST_METHOD3(processOrder, std::vector<RobotTaskInterface::Ptr>(
                                       Order& order, const std::set<AgvId>& agvs_in_use,
                                       const std::set<StationId>& stations_in_use));
};

}  // namespace tijcore
