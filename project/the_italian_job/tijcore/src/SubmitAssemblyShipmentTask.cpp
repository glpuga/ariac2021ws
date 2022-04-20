/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// standard library
#include <string>
#include <utility>

// tijcore
#include <tijcore/tasking/SubmitAssemblyShipmentTask.hpp>
#include <tijlogger/logger.hpp>

namespace tijcore
{
SubmitAssemblyShipmentTask::SubmitAssemblyShipmentTask(const Toolbox::SharedPtr& toolbox,
                                                       const std::string& assembly_tray_name,
                                                       const ShipmentType& shipment_type)
  : toolbox_{ toolbox }, assembly_tray_name_{ assembly_tray_name }, shipment_type_{ shipment_type }
{
}

RobotTaskOutcome SubmitAssemblyShipmentTask::run()
{
  auto proces_manager = toolbox_->getProcessManager();
  INFO("Submitting shipment {} on {}", shipment_type_, assembly_tray_name_);
  proces_manager->submitAssemblyStation(station_id::fromString(assembly_tray_name_),
                                        shipment_type_);
  return RobotTaskOutcome::TASK_SUCCESS;
}

void SubmitAssemblyShipmentTask::halt()
{
  WARNING(
      "halt() called on {} but the task should be too quick to require "
      "halting. halt() will do nothing.");
}

}  // namespace tijcore
