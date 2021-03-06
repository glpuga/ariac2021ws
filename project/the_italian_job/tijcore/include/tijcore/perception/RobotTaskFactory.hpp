/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// tijcore
#include <tijcore/perception/ResourceManager.hpp>
#include <tijcore/perception/RobotTaskFactoryInterface.hpp>
#include <tijcore/perception/Toolbox.hpp>

namespace tijcore {

class RobotTaskFactory : public RobotTaskFactoryInterface {
public:
  RobotTaskFactory(const ResourceManagerInterface::SharedPtr &resource_manager,
                   const Toolbox::SharedPtr toolbox);

  RobotTaskInterface::Ptr getRemoveBrokenPartTask(
      ResourceManagerInterface::ManagedLocusHandle &&source_locus,
      ResourceManagerInterface::PickAndPlaceRobotHandle &&robot) const override;

  RobotTaskInterface::Ptr getPickAndPlaceTask(
      ResourceManagerInterface::ManagedLocusHandle &&source,
      ResourceManagerInterface::ManagedLocusHandle &&destination,
      ResourceManagerInterface::PickAndPlaceRobotHandle &&robot) const override;

  RobotTaskInterface::Ptr getSubmitKittingShipmentTask(
      ResourceManagerInterface::SubmissionTrayHandle &&tray,
      const StationId &destination_station,
      const ShipmentType &shipment_type) const override;

  RobotTaskInterface::Ptr getSubmitAssemblyShipmentTask(
      ResourceManagerInterface::SubmissionTrayHandle &&tray,
      const ShipmentType &shipment_type) const override;

private:
  ResourceManagerInterface::SharedPtr resource_manager_;
  Toolbox::SharedPtr toolbox_;
};

} // namespace tijcore
