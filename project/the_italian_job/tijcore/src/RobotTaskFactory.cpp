/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <chrono>
#include <memory>
#include <string>
#include <utility>

// tijcore

#include <tijcore/tasking/PickAndPlaceTask.hpp>
#include <tijcore/tasking/RemoveBrokenPartTask.hpp>
#include <tijcore/tasking/RobotTaskFactory.hpp>
// #include <tijcore/tasking/SubmitAssemblyShipmentTask.hpp>
// #include <tijcore/tasking/SubmitKittingShipmentTask.hpp>
#include <tijlogger/logger.hpp>

namespace tijcore
{
RobotTaskFactory::RobotTaskFactory(const ResourceManagerInterface::SharedPtr& resource_manager,
                                   const Toolbox::SharedPtr toolbox)
  : resource_manager_{ resource_manager }, toolbox_{ toolbox } {};

RobotTaskInterface::Ptr RobotTaskFactory::getRemoveBrokenPartTask(
    ResourceManagerInterface::ManagedLocusHandle&& source_locus,
    ResourceManagerInterface::PickAndPlaceRobotHandle&& robot) const
{
  // return std::make_unique<RemoveBrokenPartTask>(resource_manager_, toolbox_,
  //                                               std::move(source_locus), std::move(robot));
  return nullptr;
}

RobotTaskInterface::Ptr RobotTaskFactory::getPickAndPlacePartTask(
    ResourceManagerInterface::ManagedLocusHandle&& source,
    ResourceManagerInterface::ManagedLocusHandle&& destination,
    ResourceManagerInterface::PickAndPlaceRobotHandle&& robot) const
{
  // return std::make_unique<PickAndPlaceTask>(resource_manager_, std::move(source),
  //                                           std::move(destination), std::move(robot));
  return nullptr;
}

RobotTaskInterface::Ptr RobotTaskFactory::getPickAndPlaceMovableTrayTask(
    ResourceManagerInterface::ManagedLocusHandle&& source,
    ResourceManagerInterface::ManagedLocusHandle&& destination,
    ResourceManagerInterface::PickAndPlaceRobotHandle&& robot) const
{
  return nullptr;
}

RobotTaskInterface::Ptr RobotTaskFactory::getSubmitKittingShipmentTask(
    const std::string& kitting_tray_name, const StationId& destination_station,
    const ShipmentType& shipment_type) const
{
  // if (!agv::isValid(tray.resource()->name()))
  // {
  //   throw std::invalid_argument{ tray.resource()->name() + " is not an agv id!" };
  // }
  // return std::make_unique<SubmitKittingShipmentTask>(toolbox_, std::move(tray),
  // destination_station,
  //                                                    shipment_type);
  return nullptr;
}

RobotTaskInterface::Ptr RobotTaskFactory::getSubmitAssemblyShipmentTask(
    const std::string& assembly_tray_name, const ShipmentType& shipment_type) const
{
  // if (!station_id::isValid(tray.resource()->name()))
  // {
  //   throw std::invalid_argument{ tray.resource()->name() + " is not a station id!" };
  // }
  // return std::make_unique<SubmitAssemblyShipmentTask>(toolbox_, std::move(tray), shipment_type);
  return nullptr;
}

}  // namespace tijcore
