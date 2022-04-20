/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>
#include <string>

// tijcore
#include <tijcore/abstractions/ResourceManagerInterface.hpp>
#include <tijcore/abstractions/RobotTaskInterface.hpp>
#include <tijcore/datatypes/ShipmentType.hpp>
#include <tijcore/datatypes/StationId.hpp>

namespace tijcore
{
class RobotTaskFactoryInterface
{
public:
  using Ptr = std::unique_ptr<RobotTaskFactoryInterface>;
  using SharedPtr = std::shared_ptr<RobotTaskFactoryInterface>;

  virtual ~RobotTaskFactoryInterface() = default;

  virtual RobotTaskInterface::Ptr
  getRemoveBrokenPartTask(ResourceManagerInterface::ManagedLocusHandle&& source_locus,
                          ResourceManagerInterface::PickAndPlaceRobotHandle&& robot) const = 0;

  virtual RobotTaskInterface::Ptr
  getPickAndPlacePartTask(ResourceManagerInterface::ManagedLocusHandle&& source,
                          ResourceManagerInterface::ManagedLocusHandle&& destination,
                          ResourceManagerInterface::PickAndPlaceRobotHandle&& robot) const = 0;

  virtual RobotTaskInterface::Ptr getPickAndPlaceMovableTrayTask(
      ResourceManagerInterface::ManagedLocusHandle&& source,
      ResourceManagerInterface::ManagedLocusHandle&& destination,
      ResourceManagerInterface::PickAndPlaceRobotHandle&& robot) const = 0;

  virtual RobotTaskInterface::Ptr
  getSubmitKittingShipmentTask(const std::string& kitting_tray_name, const StationId& station_id,
                               const ShipmentType& shipment_type) const = 0;

  virtual RobotTaskInterface::Ptr getSubmitAssemblyShipmentTask(
      const std::string& assembly_tray_name, const ShipmentType& shipment_type) const = 0;
};

}  // namespace tijcore
