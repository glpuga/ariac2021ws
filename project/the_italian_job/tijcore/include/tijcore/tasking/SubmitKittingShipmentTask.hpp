/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <string>

// tijcore
#include <tijcore/abstractions/ResourceManagerInterface.hpp>
#include <tijcore/abstractions/RobotTaskInterface.hpp>
#include <tijcore/coremodels/Toolbox.hpp>

namespace tijcore
{
class SubmitKittingShipmentTask : public RobotTaskInterface
{
public:
  SubmitKittingShipmentTask(const Toolbox::SharedPtr& toolbox,
                            ResourceManagerInterface::SubmissionTrayHandle&& tray,
                            const StationId& destination_station,
                            const ShipmentType& shipment_type);

  RobotTaskOutcome run() override;

  void halt() override;

private:
  Toolbox::SharedPtr toolbox_;
  std::string kitting_tray_name_;
  StationId destination_station_;
  ShipmentType shipment_type_;
};

}  // namespace tijcore
