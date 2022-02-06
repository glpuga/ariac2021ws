/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// tijcore
#include <tijcore/perception/ResourceManagerInterface.hpp>
#include <tijcore/perception/Toolbox.hpp>
#include <tijcore/tasks/RobotTaskInterface.hpp>

namespace tijcore
{
class SubmitKittingShipmentTask : public RobotTaskInterface
{
public:
  SubmitKittingShipmentTask(const Toolbox::SharedPtr& toolbox, ResourceManagerInterface::SubmissionTrayHandle&& tray,
                            const StationId& destination_station, const ShipmentType& shipment_type);

  RobotTaskOutcome run() override;

  void halt() override;

private:
  Toolbox::SharedPtr toolbox_;
  ResourceManagerInterface::SubmissionTrayHandle tray_;
  StationId destination_station_;
  ShipmentType shipment_type_;
};

}  // namespace tijcore
