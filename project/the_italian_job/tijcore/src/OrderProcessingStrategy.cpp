/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// standard library
#include <algorithm>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

// tijcore
#include <tijcore/coremodels/OrderProcessingStrategy.hpp>
#include <tijlogger/logger.hpp>

namespace tijcore
{
namespace
{
// this is the threshold for the value of the
// dot product between the z vectors of a mising piece
// and the target locus to decide that flipping is necessary.
// -0.5 basically ask that the part is 180 degrees flipped
// compared with the target locus.
const double part_flipping_threshold_ = -0.5;

const double free_radious_for_unwanted_pieces{ 0.1 };

const double free_radious_for_flippable_pieces{ 0.2 };

}  // namespace

OrderProcessingStrategy::OrderProcessingStrategy(
    const ResourceManagerInterface::SharedPtr& resource_manager,
    const RobotTaskFactoryInterface::SharedPtr& robot_task_factory,
    const Toolbox::SharedPtr& toolbox)
  : resource_manager_{ resource_manager }
  , robot_task_factory_{ robot_task_factory }
  , toolbox_{ toolbox }
{
  frame_transformer_ = toolbox_->getFrameTransformer();
}

void OrderProcessingStrategy::getAgvsAndStationsInUse(Order& order, std::set<AgvId>& agvs_in_use,
                                                      std::set<StationId>& stations_in_use) const
{
  for (auto& kshipment : order.kitting_shipments)
  {
    if (!agv::isAny(kshipment.agv_id))
    {
      agvs_in_use.emplace(kshipment.agv_id);
    }
    if (!station_id::isAny(kshipment.station_id))
    {
      stations_in_use.emplace(kshipment.station_id);
    }
  }
  for (auto& ashipment : order.assembly_shipments)
  {
    if (!station_id::isAny(ashipment.station_id))
    {
      stations_in_use.emplace(ashipment.station_id);
    }
  }
}

void OrderProcessingStrategy::disambiguateOrderEndpoints(Order& order, std::set<AgvId>& agvs_in_use,
                                                         std::set<StationId>& stations_in_use) const
{
  // TODO(glpuga) this currently does not deal with weird cases, such as there
  // not being availability in any agvN for the required asN (e.g. destination
  // is as1, but both agv1 and agv2 are in use). As it stands, this will try to
  // reuse one of the agvs, and order priorities will decide what happens.
  // Also, the case where both station and agv are "any" is not contemplated
  // here.

  for (auto& kshipment : order.kitting_shipments)
  {
    if (agv::isAny(kshipment.agv_id))
    {
      auto target_station = kshipment.station_id;

      if ((target_station == StationId::as1) || (target_station == StationId::as2))
      {
        if (!agvs_in_use.count(AgvId::agv1))
        {
          kshipment.agv_id = AgvId::agv1;
        }
        else
        {
          kshipment.agv_id = AgvId::agv2;
        }
      }
      else if ((target_station == StationId::as3) || (target_station == StationId::as4))
      {
        if (!agvs_in_use.count(AgvId::agv3))
        {
          kshipment.agv_id = AgvId::agv3;
        }
        else
        {
          kshipment.agv_id = AgvId::agv4;
        }
      }

      agvs_in_use.emplace(kshipment.agv_id);
    }

    if (station_id::isAny(kshipment.station_id))
    {
      auto source_agv = kshipment.agv_id;

      if ((source_agv == AgvId::agv1) || (source_agv == AgvId::agv2))
      {
        if (!stations_in_use.count(StationId::as1))
        {
          kshipment.station_id = StationId::as1;
        }
        else
        {
          kshipment.station_id = StationId::as2;
        }
      }
      else if ((source_agv == AgvId::agv3) || (source_agv == AgvId::agv4))
      {
        if (!stations_in_use.count(StationId::as3))
        {
          kshipment.station_id = StationId::as3;
        }
        else
        {
          kshipment.station_id = StationId::as4;
        }
      }

      stations_in_use.emplace(kshipment.station_id);
    }
  }
}

std::vector<RobotTaskInterface::Ptr>
OrderProcessingStrategy::processOrder(Order& order, const std::set<AgvId>& agvs_in_use,
                                      const std::set<StationId>& stations_in_use) const
{
  std::vector<RobotTaskInterface::Ptr> output;

  {
    auto execute_and_filter_terminated_kitting_shipments = [&](auto& kshipment) {
      auto [shipment_actions, shipment_completed] =
          processKittingShipment(order.order_id, kshipment, agvs_in_use, stations_in_use);
      std::move(shipment_actions.begin(), shipment_actions.end(), std::back_inserter(output));
      return shipment_completed;
    };

    // process each shipment, removing those that get completed
    order.kitting_shipments.erase(std::remove_if(order.kitting_shipments.begin(),
                                                 order.kitting_shipments.end(),
                                                 execute_and_filter_terminated_kitting_shipments),
                                  order.kitting_shipments.end());
  }

  {
    auto execute_and_filter_terminated_assembly_shipments = [&](auto& ashipment) {
      auto [assembly_actions, shipment_completed] =
          processAssemblyShipment(order.order_id, ashipment, agvs_in_use, stations_in_use);
      std::move(assembly_actions.begin(), assembly_actions.end(), std::back_inserter(output));
      return shipment_completed;
    };

    // process each shipment, removing those that get completed
    order.assembly_shipments.erase(std::remove_if(order.assembly_shipments.begin(),
                                                  order.assembly_shipments.end(),
                                                  execute_and_filter_terminated_assembly_shipments),
                                   order.assembly_shipments.end());
  }

  return output;
}

OrderProcessingStrategy::InitialWorldState OrderProcessingStrategy::getInitialWorldState(  //
    const std::string& target_container_name,                                              //
    const std::vector<ProductRequest>& products) const
{
  InitialWorldState initial_world_state;

  for (const auto& product : products)
  {
    const auto desired_part_id = product.type;
    const auto desired_pose = product.pose;

    // Notice that this method returns the locus regardless of whether the part is allocated or not
    auto handle_opt = resource_manager_->getLocusAtPose(desired_pose);
    if (!handle_opt)
    {
      ERROR("Failed to generate a managed locus for {}", desired_pose)
      continue;
    }

    auto& handle = *handle_opt;
    auto& locus = *handle.resource();

    // if the place is empty, store in missing parts
    if (locus.isEmptyLocus())
    {
      // notice that since I own a handle now, asking whether the handle is
      // allocated or not does not tell me if it was allocated before. Using a
      // pointer instead of optional would cause other issues.
      if (handle.allocationCount() <= 2)
      {
        // the handle we have here is the only one besides the one internally
        // stored by the resource manager.
        DEBUG("There's a missing part at {}", desired_pose);
        initial_world_state.missing_parts.emplace_back(
            std::make_pair(std::move(handle), desired_part_id));
      }
      else
      {
        // there are other handles to the same part out there
        DEBUG("There's a missing part at {}, but is currently affected to a task", desired_pose);
      }
      continue;
    }

    // if the part is broken or not the correct type, ignore it, it will be
    // dealt with later
    const auto part_id = locus.qualifiedPartInfo().part_type;
    const auto broken = locus.qualifiedPartInfo().part_is_broken;
    if (broken || (part_id != desired_part_id))
    {
      WARNING("Unwanted/Broken part at {} ({} is not {}, {})", desired_pose, part_id.codedString(),
              desired_part_id.codedString(), broken);
      continue;
    }

    // If we got here, the part is correctly located, so we keep the handle to
    // mark the piece as in use
    INFO("In-pose part at {} ({})", desired_pose, part_id.codedString());
    initial_world_state.parts_in_place.emplace_back(std::move(handle));
  }

  // Get the rest of the pieces in the same containers. This will include pieces
  // that don't belong in the shipment, as well as those that do, but are broken
  {
    // notice that this list won't contain the parts that we already have a copy of in the initial
    // world state vectors, because we are owning them
    auto parts_to_remove = resource_manager_->findSiblingLociByCommonParent(target_container_name);

    // from the parts to remove, separate broken and unwanted
    for (const auto& part : parts_to_remove)
    {
      const auto broken = part.resource()->qualifiedPartInfo().part_is_broken;
      if (broken)
      {
        initial_world_state.broken_parts.push_back(part);
      }
      else
      {
        initial_world_state.unwanted_parts.push_back(part);
      }
    }
  }

  INFO("Analysis - in_pose:{} broken:{} missing:{} unwanted:{}",
       initial_world_state.parts_in_place.size(), initial_world_state.broken_parts.size(),
       initial_world_state.missing_parts.size(), initial_world_state.unwanted_parts.size());

  return initial_world_state;
}

void OrderProcessingStrategy::removeLociInTargetSurfaces(                            //
    const std::set<AgvId>& agvs_in_use,                                              //
    const std::set<StationId>& assemblies_in_use,                                    //
    std::vector<ResourceManagerInterface::ManagedLocusHandle>& target_vector) const  //
{
  auto active_agv_and_assemblies_filter =
      [agvs_in_use,
       assemblies_in_use](const ResourceManagerInterface::ManagedLocusHandle& handle) -> bool {
    auto parent_container_name = handle.resource()->parentName();
    // TODO(glpuga) last minute hack. Ignore all assembly stations, because it's
    // using them to turn pumps around.
    if (station_id::isValid(parent_container_name))
    {
      return true;
    }
    if (agv::isValid(parent_container_name))
    {
      auto agv_id = agv::fromString(parent_container_name);
      return agvs_in_use.count(agv_id);
    }
    if (station_id::isValid(parent_container_name))
    {
      auto station_id = station_id::fromString(parent_container_name);
      return assemblies_in_use.count(station_id);
    }
    return false;
  };

  target_vector.erase(
      std::remove_if(target_vector.begin(), target_vector.end(), active_agv_and_assemblies_filter),
      target_vector.end());
}

std::vector<RobotTaskInterface::Ptr> OrderProcessingStrategy::stageRemoveBrokenParts(  //
    InitialWorldState& world_state) const                                              //
{
  // now we begin creation actions to deal with stuff
  std::vector<RobotTaskInterface::Ptr> output_actions;
  for (auto& part : world_state.broken_parts)
  {
    auto robot_handle_opt = resource_manager_->getPickAndPlaceRobotHandle();
    if (robot_handle_opt)
    {
      INFO("Creating a RemoveBrokenPartTask for {} for {}", robot_handle_opt->resource()->name(),
           part.resource()->pose());
      output_actions.emplace_back(robot_task_factory_->getRemoveBrokenPartTask(
          std::move(part), std::move(*robot_handle_opt)));
    }
  }
  return output_actions;
}

std::vector<RobotTaskInterface::Ptr> OrderProcessingStrategy::stageRemoveUnwantedParts(  //
    const std::set<AgvId>& agvs_in_use,                                                  //
    const std::set<StationId>& assemblies_in_use,                                        //
    InitialWorldState& world_state) const                                                //
{
  std::vector<RobotTaskInterface::Ptr> output_actions;
  std::vector<ResourceManagerInterface::ManagedLocusHandle> empty_loci;

  for (auto& part : world_state.unwanted_parts)
  {
    if (empty_loci.empty())
    {
      // try to get a new list of empty loci
      empty_loci = resource_manager_->findVacantLociCandidates(free_radious_for_unwanted_pieces);

      // remove loci in agvs that are currently targeted by orders
      removeLociInTargetSurfaces(agvs_in_use, assemblies_in_use, empty_loci);

      if (empty_loci.empty())
      {
        // there are no more empty spaces!
        WARNING("We ran out of empty loci in the environment!")
        break;
      }
    }

    // sort by distance to the part, so that we pick closer ones first
    stableSortByDistanceToReferencePose(part.resource()->pose(), empty_loci);

    while (empty_loci.size())
    {
      auto last_it = empty_loci.end() - 1;
      auto closest_empty_spot = std::move(*last_it);
      empty_loci.erase(last_it);

      auto robot_handle_opt = resource_manager_->getPickAndPlaceRobotHandle();

      if (robot_handle_opt)
      {
        WARNING(
            "Creating a PickAndPlaceTask for {} to move an unwanted piece "
            "from {} to {}",
            robot_handle_opt->resource()->name(), part.resource()->pose(),
            closest_empty_spot.resource()->pose());

        output_actions.emplace_back(robot_task_factory_->getPickAndPlaceTask(
            std::move(part), std::move(closest_empty_spot), std::move(*robot_handle_opt)));
        break;
      }
    }
  }

  return output_actions;
}

std::vector<RobotTaskInterface::Ptr> OrderProcessingStrategy::stagePlaceMissingParts(  //
    const std::set<AgvId>& agvs_in_use,                                                //
    const std::set<StationId>& assemblies_in_use,                                      //
    InitialWorldState& world_state,                                                    //
    int32_t& unavailable_part_count) const                                             //
{
  std::vector<RobotTaskInterface::Ptr> output_actions;

  for (auto& [missing_part_locus, part_id] : world_state.missing_parts)
  {
    auto potential_sources = resource_manager_->getPartSourceListByType(part_id);

    // remove parts in agvs that are currently targeted by orders
    removeLociInTargetSurfaces(agvs_in_use, assemblies_in_use, potential_sources);

    // sort by distance (larger distance first, to remove efficiently the
    // nearest spot from the list)
    stableSortByDistanceToReferencePose(missing_part_locus.resource()->pose(), potential_sources);

    // Choose the first one of the remaining lot, if any
    if (potential_sources.empty())
    {
      ++unavailable_part_count;
    }
    else
    {
      // select the closest part
      auto& selected_source_part = *(potential_sources.end() - 1);

      auto robot_handle_opt = resource_manager_->getPickAndPlaceRobotHandle();

      if (robot_handle_opt)
      {
        WARNING(
            "Creating a PickAndPlaceTask for {} to provide a part from "
            "{} and into {}",
            robot_handle_opt->resource()->name(), selected_source_part.resource()->pose(),
            missing_part_locus.resource()->pose());
        output_actions.emplace_back(robot_task_factory_->getPickAndPlaceTask(
            std::move(selected_source_part), std::move(missing_part_locus),
            std::move(*robot_handle_opt)));
      }
    }
  }

  return output_actions;
}

std::vector<RobotTaskInterface::Ptr> OrderProcessingStrategy::stageSubmitShipping(  //
    const std::string& target_container_name,                                       //
    const ShipmentClass shipment_class,                                             //
    const ShipmentType& shipment_type,                                              //
    const StationId& station_id,                                                    //
    InitialWorldState& world_state,                                                 //
    bool& shipping_done) const                                                      //
{
  std::vector<RobotTaskInterface::Ptr> output_actions;

  // release these pieces or we won't be able to capture the tray
  world_state.parts_in_place.clear();
  world_state.missing_parts.clear();

  shipping_done = true;

  switch (shipment_class)
  {
    case ShipmentClass::Kitting:
      WARNING("Creating a SubmitKittingShipmentTask for {}", target_container_name);
      output_actions.emplace_back(robot_task_factory_->getSubmitKittingShipmentTask(
          target_container_name, station_id, shipment_type));
      break;
    case ShipmentClass::Assembly:
      WARNING("Creating a SubmitAssemblyShipmentTask for {}", target_container_name);
      output_actions.emplace_back(
          robot_task_factory_->getSubmitAssemblyShipmentTask(target_container_name, shipment_type));
      break;
  }
  return output_actions;
}

std::pair<std::vector<RobotTaskInterface::Ptr>, bool>
OrderProcessingStrategy::processKittingShipment(const OrderId& order,
                                                const KittingShipment& shipment,
                                                const std::set<AgvId>& agvs_in_use,
                                                const std::set<StationId>& stations_in_use) const
{
  const std::string target_container_name = agv::toString(shipment.agv_id);

  auto world_state = getInitialWorldState(target_container_name, shipment.products);

  //
  // with highest priority of all, we need to remove broken pieces
  if (world_state.broken_parts.size())
  {
    return std::make_pair(stageRemoveBrokenParts(world_state), false);
  }

  // next deal with unwanted pieces present in the container. Relocate them
  // somewhere else.
  if (world_state.unwanted_parts.size())
  {
    return std::make_pair(stageRemoveUnwantedParts(agvs_in_use, stations_in_use, world_state),
                          false);
  }

  int32_t unavailable_part_count{ 0 };

  // only move parts into the container if there's nothing moving out
  if (world_state.missing_parts.size())
  {
    return std::make_pair(
        stagePlaceMissingParts(agvs_in_use, stations_in_use, world_state, unavailable_part_count),
        false);
  }

  bool shipping_done{ false };

  // if there's nothing missing, then submit the shipping
  if ((world_state.broken_parts.size() == 0) && (world_state.unwanted_parts.size() == 0) &&
      (shipment.products.size() == world_state.parts_in_place.size() + unavailable_part_count))
  {
    return std::make_pair(stageSubmitShipping(target_container_name, ShipmentClass::Kitting,
                                              shipment.shipment_type, shipment.station_id,
                                              world_state, shipping_done),
                          false);
  }

  return std::make_pair(std::vector<RobotTaskInterface::Ptr>{}, false);
}

std::pair<std::vector<RobotTaskInterface::Ptr>, bool>
OrderProcessingStrategy::processAssemblyShipment(const OrderId& order,
                                                 const AssemblyShipment& shipment,
                                                 const std::set<AgvId>& agvs_in_use,
                                                 const std::set<StationId>& stations_in_use) const
{
  const std::string target_container_name = station_id::toString(shipment.station_id);

  auto world_state = getInitialWorldState(target_container_name, shipment.products);

  //
  // with highest priority of all, we need to remove broken pieces
  if (world_state.broken_parts.size())
  {
    return std::make_pair(stageRemoveBrokenParts(world_state), false);
  }

  // next deal with unwanted pieces present in the container. Relocate them
  // somewhere else.
  if (world_state.unwanted_parts.size())
  {
    return std::make_pair(stageRemoveUnwantedParts(agvs_in_use, stations_in_use, world_state),
                          false);
  }

  int32_t unavailable_part_count{ 0 };

  // only move parts into the container if there's nothing moving out
  if (world_state.missing_parts.size())
  {
    return std::make_pair(
        stagePlaceMissingParts(agvs_in_use, stations_in_use, world_state, unavailable_part_count),
        false);
  }

  bool shipping_done{ false };

  // if there's nothing missing, then submit the shipping
  if ((world_state.broken_parts.size() == 0) && (world_state.unwanted_parts.size() == 0) &&
      (shipment.products.size() == world_state.parts_in_place.size() + unavailable_part_count))
  {
    return std::make_pair(stageSubmitShipping(target_container_name, ShipmentClass::Assembly,
                                              shipment.shipment_type, shipment.station_id,
                                              world_state, shipping_done),
                          false);
  }

  return std::make_pair(std::vector<RobotTaskInterface::Ptr>{}, false);
}

void OrderProcessingStrategy::stableSortByDistanceToReferencePose(                        //
    const tijmath::RelativePose3& reference_pose,                                         //
    std::vector<ResourceManagerInterface::ManagedLocusHandle>& target_loci_vector) const  //
{
  auto sort_farthest_first_generalized =
      [this](const tijmath::RelativePose3& reference_pose,
             const ResourceManagerInterface::ManagedLocusHandle& lhs,
             const ResourceManagerInterface::ManagedLocusHandle& rhs) {
        const auto& lhs_pose = lhs.resource()->pose();
        const auto& rhs_pose = rhs.resource()->pose();
        // get all the poses in the same reference frame
        const auto target_lhs_pose =
            frame_transformer_->transformPoseToFrame(lhs_pose, reference_pose.frameId());
        const auto target_rhs_pose =
            frame_transformer_->transformPoseToFrame(rhs_pose, reference_pose.frameId());

        const auto ref_z = reference_pose.rotation().rotationMatrix().col(2);
        const auto lhs_z = lhs_pose.rotation().rotationMatrix().col(2);
        const auto rhs_z = rhs_pose.rotation().rotationMatrix().col(2);
        const auto lhs_proj = lhs_z.dot(ref_z);
        const auto rhs_proj = rhs_z.dot(ref_z);

        // if the projections have different signs, then order higher in
        // priority the part that is aligned with the reference
        if (lhs_proj * rhs_proj < 0)
        {
          return lhs_proj < 0;
        }

        // finally, given all else equal, order by distance, giving preference
        // to parts that are closer to the reference
        const auto lhs_distance =
            (reference_pose.position().vector() - target_lhs_pose.position().vector()).norm();
        const auto rhs_distance =
            (reference_pose.position().vector() - target_rhs_pose.position().vector()).norm();

        return lhs_distance > rhs_distance;
      };

  auto sort_farthest_first = [this, sort_farthest_first_generalized, reference_pose](
                                 const ResourceManagerInterface::ManagedLocusHandle& lhs,
                                 const ResourceManagerInterface::ManagedLocusHandle& rhs) {
    return sort_farthest_first_generalized(reference_pose, lhs, rhs);
  };

  // sort by distance (larger distance first, to remove efficiently the
  // nearest spot from the list)
  std::stable_sort(target_loci_vector.begin(), target_loci_vector.end(), sort_farthest_first);
}

}  // namespace tijcore
