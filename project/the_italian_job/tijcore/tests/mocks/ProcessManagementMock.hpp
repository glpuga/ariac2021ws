/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>
#include <set>
#include <string>
#include <vector>

// gtest
#include "gmock/gmock.h"

// tijcore
#include <tijcore/abstractions/ProcessManagementInterface.hpp>

namespace tijcore
{
class ProcessManagementMock : public ProcessManagementInterface
{
public:
  using Ptr = std::unique_ptr<ProcessManagementMock>;
  using SharedPtr = std::shared_ptr<ProcessManagementMock>;

  MOCK_CONST_METHOD0(startCompetition, ErrorWithReason());

  MOCK_CONST_METHOD0(endCompetition, ErrorWithReason());

  MOCK_CONST_METHOD0(getCompetitionState, std::string());

  MOCK_METHOD0(getOrders, std::vector<Order>());

  MOCK_CONST_METHOD3(submitAgvToAssemblyStation, ErrorWithReason(const AgvId& agv, const StationId& destination_station,
                                                                 const std::string& shipment_type));

  MOCK_CONST_METHOD1(getAgvState, std::string(const AgvId& agv_id));

  MOCK_CONST_METHOD2(submitAssemblyStation,
                     ErrorWithReason(const StationId& station, const std::string& shipment_type));

  MOCK_CONST_METHOD1(getAgvStation, StationId(const AgvId& agv_id));
};

}  // namespace tijcore
