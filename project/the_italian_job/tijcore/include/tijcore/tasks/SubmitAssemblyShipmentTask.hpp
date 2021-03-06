/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// tijcore
#include <tijcore/perception/ResourceManagerInterface.hpp>
#include <tijcore/perception/Toolbox.hpp>
#include <tijcore/tasks/RobotTaskInterface.hpp>

namespace tijcore {

class SubmitAssemblyShipmentTask : public RobotTaskInterface {
public:
  SubmitAssemblyShipmentTask(
      const Toolbox::SharedPtr &toolbox,
      ResourceManagerInterface::SubmissionTrayHandle &&tray,
      const ShipmentType &shipment_type);

  RobotTaskOutcome run() override;

  void halt() override;

private:
  const Toolbox::SharedPtr toolbox_;
  ResourceManagerInterface::SubmissionTrayHandle tray_;
  ShipmentType shipment_type_;
};

} // namespace tijcore
