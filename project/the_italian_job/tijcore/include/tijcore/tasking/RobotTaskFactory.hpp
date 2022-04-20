/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <string>

// tijcore
#include <tijcore/abstractions/RobotTaskFactoryInterface.hpp>
#include <tijcore/coremodels/Toolbox.hpp>
#include <tijcore/resources/ResourceManager.hpp>
namespace tijcore
{
class RobotTaskFactory : public RobotTaskFactoryInterface
{
public:
  RobotTaskFactory(const ResourceManagerInterface::SharedPtr& resource_manager,
                   const Toolbox::SharedPtr toolbox);

  RobotTaskInterface::Ptr
  getRemoveBrokenPartTask(ResourceManagerInterface::ManagedLocusHandle&& source_locus,
                          ResourceManagerInterface::PickAndPlaceRobotHandle&& robot) const override;

  RobotTaskInterface::Ptr
  getPickAndPlacePartTask(ResourceManagerInterface::ManagedLocusHandle&& source,
                          ResourceManagerInterface::ManagedLocusHandle&& destination,
                          ResourceManagerInterface::PickAndPlaceRobotHandle&& robot) const override;

  RobotTaskInterface::Ptr getPickAndPlaceMovableTrayTask(
      ResourceManagerInterface::ManagedLocusHandle&& source,
      ResourceManagerInterface::ManagedLocusHandle&& destination,
      ResourceManagerInterface::PickAndPlaceRobotHandle&& robot) const override;

  RobotTaskInterface::Ptr getSubmitKittingShipmentTask(
      const std::string& kitting_tray_name, const StationId& destination_station,
      const ShipmentType& shipment_type) const override;

  RobotTaskInterface::Ptr getSubmitAssemblyShipmentTask(
      const std::string& assembly_tray_name, const ShipmentType& shipment_type) const override;

private:
  ResourceManagerInterface::SharedPtr resource_manager_;
  Toolbox::SharedPtr toolbox_;
};

}  // namespace tijcore
