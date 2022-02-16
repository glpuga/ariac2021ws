/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>
#include <string>
#include <utility>
#include <vector>

// tijcore
#include <tijcore/datatypes/Order.hpp>

namespace tijcore
{
class ProcessManagementInterface
{
public:
  using Ptr = std::unique_ptr<ProcessManagementInterface>;
  using SharedPtr = std::shared_ptr<ProcessManagementInterface>;

  using ErrorWithReason = std::pair<bool, std::string>;

  virtual ~ProcessManagementInterface() = default;

  virtual ErrorWithReason startCompetition() const = 0;

  virtual ErrorWithReason endCompetition() const = 0;

  virtual std::string getCompetitionState() const = 0;

  virtual std::vector<Order> getOrders() = 0;

  virtual ErrorWithReason submitAgvToAssemblyStation(const AgvId& agv, const StationId& destination_station,
                                                     const std::string& shipment_type) const = 0;

  virtual std::string getAgvState(const AgvId& agv) const = 0;

  virtual ErrorWithReason submitAssemblyStation(const StationId& station, const std::string& shipment_type) const = 0;

  virtual StationId getAgvStation(const AgvId& station) const = 0;
};

}  // namespace tijcore
