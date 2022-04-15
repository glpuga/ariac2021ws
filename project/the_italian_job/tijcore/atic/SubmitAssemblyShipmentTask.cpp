/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// standard library
#include <utility>

// tijcore
#include <tijcore/tasking/SubmitAssemblyShipmentTask.hpp>
#include <tijlogger/logger.hpp>

namespace tijcore
{
SubmitAssemblyShipmentTask::SubmitAssemblyShipmentTask(
    const Toolbox::SharedPtr& toolbox, ResourceManagerInterface::SubmissionTrayHandle&& tray,
    const ShipmentType& shipment_type)
  : toolbox_{ toolbox }, tray_{ std::move(tray) }, shipment_type_{ shipment_type }
{
}

RobotTaskOutcome SubmitAssemblyShipmentTask::run()
{
  auto proces_manager = toolbox_->getProcessManager();
  INFO("Submitting shipment {} on {}", shipment_type_, tray_.resource()->name());
  proces_manager->submitAssemblyStation(station_id::fromString(tray_.resource()->name()),
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
