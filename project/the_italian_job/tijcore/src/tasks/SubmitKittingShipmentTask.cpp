/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// standard library
#include <utility>

// tijcore
#include <logger/logger.hpp>
#include <tijcore/tasks/SubmitKittingShipmentTask.hpp>

namespace tijcore {

SubmitKittingShipmentTask::SubmitKittingShipmentTask(
    const Toolbox::SharedPtr &toolbox,
    ResourceManagerInterface::SubmissionTrayHandle &&tray,
    const StationId &destination_station, const ShipmentType &shipment_type)
    : toolbox_{toolbox}, tray_{std::move(tray)},
      destination_station_{destination_station}, shipment_type_{shipment_type} {
}

RobotTaskOutcome SubmitKittingShipmentTask::run() {
  auto proces_manager = toolbox_->getProcessManager();
  INFO("Submitting shipment {} on {}", shipment_type_,
       tray_.resource()->name());
  proces_manager->submitAgvToAssemblyStation(
      agv::fromString(tray_.resource()->name()), destination_station_,
      shipment_type_);
  return RobotTaskOutcome::TASK_SUCCESS;
}

void SubmitKittingShipmentTask::halt() {
  WARNING("halt() called on {} but the task should be too quick to require "
          "halting. halt() will do nothing.");
}

} // namespace tijcore
