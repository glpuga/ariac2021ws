/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// standard library
#include <chrono>
#include <string>
#include <thread>
#include <utility>

// tijcore
#include <tijcore/tasking/SubmitKittingShipmentTask.hpp>
#include <tijlogger/logger.hpp>

namespace tijcore
{
namespace
{
std::chrono::seconds anti_ghosting_delay{ 10 };
}

SubmitKittingShipmentTask::SubmitKittingShipmentTask(const Toolbox::SharedPtr& toolbox,
                                                     const std::string& kitting_tray_name,
                                                     const StationId& destination_station,
                                                     const ShipmentType& shipment_type)
  : toolbox_{ toolbox }
  , kitting_tray_name_{ kitting_tray_name }
  , destination_station_{ destination_station }
  , shipment_type_{ shipment_type }
{
}

RobotTaskOutcome SubmitKittingShipmentTask::run()
{
  auto proces_manager = toolbox_->getProcessManager();
  INFO("Submitting shipment {} on {}", shipment_type_, kitting_tray_name_);
  proces_manager->submitAgvToAssemblyStation(agv::fromString(kitting_tray_name_),
                                             destination_station_, shipment_type_);

  INFO("The kitting shipment is on the move...");

  std::this_thread::sleep_for(anti_ghosting_delay);

  return RobotTaskOutcome::TASK_SUCCESS;
}

void SubmitKittingShipmentTask::halt()
{
  WARNING(
      "halt() called on {} but the task should be too quick to require "
      "halting. halt() will do nothing.");
}

}  // namespace tijcore
