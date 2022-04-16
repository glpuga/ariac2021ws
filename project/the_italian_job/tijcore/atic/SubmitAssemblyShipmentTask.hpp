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
class SubmitAssemblyShipmentTask : public RobotTaskInterface
{
public:
  SubmitAssemblyShipmentTask(const Toolbox::SharedPtr& toolbox,
                             const std::string& assembly_tray_name,
                             const ShipmentType& shipment_type);

  RobotTaskOutcome run() override;

  void halt() override;

private:
  const Toolbox::SharedPtr toolbox_;
  std::string assembly_tray_name_;
  ShipmentType shipment_type_;
};

}  // namespace tijcore
