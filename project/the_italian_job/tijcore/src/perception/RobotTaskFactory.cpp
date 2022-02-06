/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library
#include <chrono>
#include <memory>
#include <utility>

// tijcore
#include <tijcore/logger/logger.hpp>
#include <tijcore/perception/RobotTaskFactory.hpp>
#include <tijcore/tasks/PickAndPlaceTask.hpp>
#include <tijcore/tasks/PickAndTwistPartTask.hpp>
#include <tijcore/tasks/RemoveBrokenPartTask.hpp>
#include <tijcore/tasks/SubmitAssemblyShipmentTask.hpp>
#include <tijcore/tasks/SubmitKittingShipmentTask.hpp>

namespace tijcore
{
RobotTaskFactory::RobotTaskFactory(const ResourceManagerInterface::SharedPtr& resource_manager,
                                   const Toolbox::SharedPtr toolbox)
  : resource_manager_{ resource_manager }, toolbox_{ toolbox } {};

RobotTaskInterface::Ptr
RobotTaskFactory::getRemoveBrokenPartTask(ResourceManagerInterface::ManagedLocusHandle&& source_locus,
                                          ResourceManagerInterface::PickAndPlaceRobotHandle&& robot) const
{
  return std::make_unique<RemoveBrokenPartTask>(resource_manager_, toolbox_, std::move(source_locus), std::move(robot));
}

RobotTaskInterface::Ptr RobotTaskFactory::getPickAndPlaceTask(
    ResourceManagerInterface::ManagedLocusHandle&& source, ResourceManagerInterface::ManagedLocusHandle&& destination,
    ResourceManagerInterface::PickAndPlaceRobotHandle&& robot) const
{
  return std::make_unique<PickAndPlaceTask>(resource_manager_, std::move(source), std::move(destination),
                                            std::move(robot));
}

RobotTaskInterface::Ptr RobotTaskFactory::getPickAndTwistPartTask(
    ResourceManagerInterface::ManagedLocusHandle&& target, ResourceManagerInterface::ManagedLocusHandle&& destination,
    ResourceManagerInterface::PickAndPlaceRobotHandle&& robot) const
{
  return std::make_unique<PickAndTwistPartTask>(resource_manager_, std::move(target), std::move(destination),
                                                std::move(robot));
}

RobotTaskInterface::Ptr RobotTaskFactory::getSubmitKittingShipmentTask(
    ResourceManagerInterface::SubmissionTrayHandle&& tray, const StationId& destination_station,
    const ShipmentType& shipment_type) const
{
  if (!agv::isValid(tray.resource()->name()))
  {
    throw std::invalid_argument{ tray.resource()->name() + " is not an agv id!" };
  }
  return std::make_unique<SubmitKittingShipmentTask>(toolbox_, std::move(tray), destination_station, shipment_type);
}

RobotTaskInterface::Ptr RobotTaskFactory::getSubmitAssemblyShipmentTask(
    ResourceManagerInterface::SubmissionTrayHandle&& tray, const ShipmentType& shipment_type) const
{
  if (!station_id::isValid(tray.resource()->name()))
  {
    throw std::invalid_argument{ tray.resource()->name() + " is not a station id!" };
  }
  return std::make_unique<SubmitAssemblyShipmentTask>(toolbox_, std::move(tray), shipment_type);
}

}  // namespace tijcore
