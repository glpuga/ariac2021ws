/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>

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

  virtual RobotTaskInterface::Ptr getPickAndPlaceTask(
      ResourceManagerInterface::ManagedLocusHandle&& source, ResourceManagerInterface::ManagedLocusHandle&& destination,
      ResourceManagerInterface::PickAndPlaceRobotHandle&& robot) const = 0;

  virtual RobotTaskInterface::Ptr getPickAndTwistPartTask(
      ResourceManagerInterface::ManagedLocusHandle&& target, ResourceManagerInterface::ManagedLocusHandle&& destination,
      ResourceManagerInterface::PickAndPlaceRobotHandle&& robot) const = 0;

  virtual RobotTaskInterface::Ptr getSubmitKittingShipmentTask(ResourceManagerInterface::SubmissionTrayHandle&& tray,
                                                               const StationId& station_id,
                                                               const ShipmentType& shipment_type) const = 0;

  virtual RobotTaskInterface::Ptr getSubmitAssemblyShipmentTask(ResourceManagerInterface::SubmissionTrayHandle&& tray,
                                                                const ShipmentType& shipment_type) const = 0;
};

}  // namespace tijcore
