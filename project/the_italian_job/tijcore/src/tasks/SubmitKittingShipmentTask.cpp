/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// standard library
#include <chrono>
#include <thread>
#include <utility>

// tijcore
#include <logger/logger.hpp>
#include <tijcore/tasks/SubmitKittingShipmentTask.hpp>

namespace tijcore {

namespace {
std::chrono::seconds anti_ghosting_delay{5};
}

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

  INFO("The kitting shipment is on the move...");

  std::this_thread::sleep_for(anti_ghosting_delay);

  return RobotTaskOutcome::TASK_SUCCESS;
}

void SubmitKittingShipmentTask::halt() {
  WARNING("halt() called on {} but the task should be too quick to require "
          "halting. halt() will do nothing.");
}

} // namespace tijcore
