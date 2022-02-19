/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// tijcore
#include <tijcore/abstractions/ResourceManagerInterface.hpp>
#include <tijcore/abstractions/RobotTaskInterface.hpp>
#include <tijcore/coremodels/Toolbox.hpp>

namespace tijcore
{
class SubmitAssemblyShipmentTask : public RobotTaskInterface
{
public:
  SubmitAssemblyShipmentTask(const Toolbox::SharedPtr& toolbox,
                             ResourceManagerInterface::SubmissionTrayHandle&& tray,
                             const ShipmentType& shipment_type);

  RobotTaskOutcome run() override;

  void halt() override;

private:
  const Toolbox::SharedPtr toolbox_;
  ResourceManagerInterface::SubmissionTrayHandle tray_;
  ShipmentType shipment_type_;
};

}  // namespace tijcore
