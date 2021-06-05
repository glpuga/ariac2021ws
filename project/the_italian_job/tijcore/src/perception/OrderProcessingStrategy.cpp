/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// standard library
#include <algorithm>
#include <memory>
#include <set>
#include <tuple>
#include <utility>

// tijcore
#include <logger/logger.hpp>
#include <tijcore/perception/OrderProcessingStrategy.hpp>

namespace tijcore {

namespace {

// this is the threshold for the value of the
// dot product between the z vectors of a mising piece
// and the target locus to decide that flipping is necessary.
// -0.5 basically ask that the part is 180 degrees flipped
// compared with the target locus.
const double part_flipping_threshold_ = -0.5;

const double free_radious_for_unwanted_pieces{0.1};

const double free_radious_for_flippable_pieces{0.2};

} // namespace

OrderProcessingStrategy::OrderProcessingStrategy(
    const ResourceManagerInterface::SharedPtr &resource_manager,
    const RobotTaskFactoryInterface::SharedPtr &robot_task_factory,
    const Toolbox::SharedPtr &toolbox)
    : resource_manager_{resource_manager},
      robot_task_factory_{robot_task_factory}, toolbox_{toolbox} {
  frame_transformer_ = toolbox_->getFrameTransformer();
}

void OrderProcessingStrategy::getAgvsAndStationsInUse(
    Order &order, std::set<AgvId> &agvs_in_use,
    std::set<StationId> &stations_in_use) const {
  for (auto &kshipment : order.kitting_shipments) {
    if (!agv::isAny(kshipment.agv_id)) {
      agvs_in_use.emplace(kshipment.agv_id);
    }
    if (!station_id::isAny(kshipment.station_id)) {
      stations_in_use.emplace(kshipment.station_id);
    }
  }
  for (auto &ashipment : order.assembly_shipments) {
    if (!station_id::isAny(ashipment.station_id)) {
      stations_in_use.emplace(ashipment.station_id);
    }
  }
}

void OrderProcessingStrategy::disambiguateOrderEndpoints(
    Order &order, std::set<AgvId> &agvs_in_use,
    std::set<StationId> &stations_in_use) const {

  // TODO(glpuga) this currently does not deal with weird cases, such as there
  // not being availability in any agvN for the required asN (e.g. destination
  // is as1, but both agv1 and agv2 are in use). As it stands, this will try to
  // reuse one of the agvs, and order priorities will decide what happens.
  // Also, the case where both station and agv are "any" is not contemplated
  // here.

  for (auto &kshipment : order.kitting_shipments) {
    if (agv::isAny(kshipment.agv_id)) {
      auto target_station = kshipment.station_id;

      if ((target_station == StationId::as1) ||
          (target_station == StationId::as2)) {
        if (!agvs_in_use.count(AgvId::agv1)) {
          kshipment.agv_id = AgvId::agv1;
        } else {
          kshipment.agv_id = AgvId::agv2;
        }
      } else if ((target_station == StationId::as3) ||
                 (target_station == StationId::as4)) {
        if (!agvs_in_use.count(AgvId::agv3)) {
          kshipment.agv_id = AgvId::agv3;
        } else {
          kshipment.agv_id = AgvId::agv4;
        }
      }

      agvs_in_use.emplace(kshipment.agv_id);
    }

    if (station_id::isAny(kshipment.station_id)) {
      auto source_agv = kshipment.agv_id;

      if ((source_agv == AgvId::agv1) || (source_agv == AgvId::agv2)) {
        if (!stations_in_use.count(StationId::as1)) {
          kshipment.station_id = StationId::as1;
        } else {
          kshipment.station_id = StationId::as2;
        }
      } else if ((source_agv == AgvId::agv3) || (source_agv == AgvId::agv4)) {
        if (!stations_in_use.count(StationId::as3)) {
          kshipment.station_id = StationId::as3;
        } else {
          kshipment.station_id = StationId::as4;
        }
      }

      stations_in_use.emplace(kshipment.station_id);
    }
  }
}

std::vector<RobotTaskInterface::Ptr> OrderProcessingStrategy::processOrder(
    Order &order, const std::set<AgvId> &agvs_in_use,
    const std::set<StationId> &stations_in_use) const {
  std::vector<RobotTaskInterface::Ptr> output;

  {
    auto execute_and_filter_terminated_assembly_shipments =
        [&](auto &ashipment) {
          auto [assembly_actions, shipment_completed] = processAssemblyShipment(
              order.order_id, ashipment, agvs_in_use, stations_in_use);
          std::move(assembly_actions.begin(), assembly_actions.end(),
                    std::back_inserter(output));
          return shipment_completed;
        };

    // process each shipment, removing those that get completed
    order.assembly_shipments.erase(
        std::remove_if(order.assembly_shipments.begin(),
                       order.assembly_shipments.end(),
                       execute_and_filter_terminated_assembly_shipments),
        order.assembly_shipments.end());
  }

  {
    auto execute_and_filter_terminated_kitting_shipments =
        [&](auto &kshipment) {
          auto [shipment_actions, shipment_completed] = processKittingShipment(
              order.order_id, kshipment, agvs_in_use, stations_in_use);
          std::move(shipment_actions.begin(), shipment_actions.end(),
                    std::back_inserter(output));
          return shipment_completed;
        };

    // process each shipment, removing those that get completed
    order.kitting_shipments.erase(
        std::remove_if(order.kitting_shipments.begin(),
                       order.kitting_shipments.end(),
                       execute_and_filter_terminated_kitting_shipments),
        order.kitting_shipments.end());
  }

  return output;
}

std::pair<std::vector<RobotTaskInterface::Ptr>, bool>
OrderProcessingStrategy::processUniversalShipment(
    const ShipmentClass shipment_class, const OrderId &order,
    const ShipmentType &shipment_type, const std::optional<AgvId> &agv_id,
    const StationId &station_id, const std::vector<ProductRequest> &products,
    const std::set<AgvId> &agvs_in_use,
    const std::set<StationId> &assemblies_in_use) const {
  const std::string target_container_name =
      shipment_class == ShipmentClass::Assembly
          ? station_id::toString(station_id)
          : agv::toString(agv_id.value());
  const std::string target_frame_id =
      resource_manager_->getContainerFrameId(target_container_name);

  std::vector<ResourceManagerInterface::ManagedLocusHandle> parts_in_place;
  std::vector<std::pair<ResourceManagerInterface::ManagedLocusHandle, PartId>>
      missing_parts;

  for (const auto &product : products) {
    const auto desired_part_id = product.type;
    const auto desired_pose = product.pose;

    auto handle_opt =
        resource_manager_->getManagedLocusHandleForPose(desired_pose);

    if (!handle_opt) {
      ERROR("Failed to generate a managed locus for {}", desired_pose)
      continue;
    }
    auto &handle = *handle_opt;

    auto &locus = *handle.resource();

    // if the place is empty, store in missing parts
    if (locus.isEmpty()) {
      // notice that since I own a handle now, asking whether the handle is
      // allocated or not does not tell me if it was allocated before. Using a
      // pointer instead of optional would cause other issues.
      if (handle.allocationCount() <= 2) {
        // the handle we have here is the only one besides the one internally
        // stored by the resource manager.
        DEBUG("There's a missing part at {}", desired_pose);
        missing_parts.emplace_back(
            std::make_pair(std::move(handle), desired_part_id));
      } else {
        // there are other handles to the same part out there
        DEBUG(
            "There's a missing part at {}, but is currently affected to a task",
            desired_pose);
      }
      continue;
    }

    // if the part is broken or not the correct type, ignore it, it will be
    // dealt with later
    auto [part_id, broken] = locus.model();
    (void)broken; // avoids unused variable warning
    if (broken || (part_id != desired_part_id)) {
      WARNING("Unwanted/Broken part at {} ({} is not {}, {})", desired_pose,
              part_id.codedString(), desired_part_id.codedString(), broken);
      continue;
    }

    // TODO(glpuga) eventually, we can discard here if the rotation is not
    // correct
    // ...

    // If we got here, the part is correctly located, so we keep the handle to
    // mark the piece as in use
    INFO("In-pose part at {} ({})", desired_pose, part_id.codedString());
    parts_in_place.emplace_back(std::move(handle));
  }

  // Get the rest of the pieces in the same containers. This will include pieces
  // that don't belong in the shipment, as well as those that do, but are broken

  std::vector<ResourceManagerInterface::ManagedLocusHandle> broken_parts;
  std::vector<ResourceManagerInterface::ManagedLocusHandle> unwanted_parts;

  {
    auto parts_to_remove =
        resource_manager_->findManagedLociByParent(target_container_name);

    // from the parts to remove, separate broken and unwanted
    for (const auto &part : parts_to_remove) {
      auto [part_id, broken] = part.resource()->model();
      (void)part_id; // avoids unused variable warning
      if (broken) {
        broken_parts.push_back(part);
      } else {
        unwanted_parts.push_back(part);
      }
    }
  }

  INFO("Analysis - in_pose:{} broken:{} missing:{} unwanted:{}",
       parts_in_place.size(), broken_parts.size(), missing_parts.size(),
       unwanted_parts.size());

  auto active_agv_and_assemblies_filter =
      [agvs_in_use, assemblies_in_use](
          const ResourceManagerInterface::ManagedLocusHandle &handle) -> bool {
    auto parent_container_name = handle.resource()->parentName();
    if (agv::isValid(parent_container_name)) {
      auto agv_id = agv::fromString(parent_container_name);
      return agvs_in_use.count(agv_id);
    }
    if (station_id::isValid(parent_container_name)) {
      auto station_id = station_id::fromString(parent_container_name);
      return assemblies_in_use.count(station_id);
    }
    return false;
  };

  // now we begin creation actions to deal with stuff
  std::vector<RobotTaskInterface::Ptr> output_actions;

  // only enter this section if no previous section generated actions
  if (broken_parts.size()) {
    // begin by dealing with broken parts. We need the piece handle and a robot
    // to do it.
    for (auto &part : broken_parts) {
      auto region = resource_manager_->getWorkRegionId(part);
      // the kitting near_bin will force the use of the kitting robot unless
      // the kitting robot is disabled. It's all part of a big hack to bias
      // the control software towards using kitting when possible.
      auto robot_handle_opt = resource_manager_->getPickAndPlaceRobotHandle(
          std::set<WorkRegionId>{region, WorkRegionId::kitting_near_bins});

      if (robot_handle_opt) {
        WARNING("Creating a RemoveBrokenPartTask for {} for {}",
                robot_handle_opt->resource()->name(), part.resource()->pose());
        output_actions.emplace_back(
            robot_task_factory_->getRemoveBrokenPartTask(
                std::move(part), std::move(*robot_handle_opt)));
      }
    }
  }

  auto sort_farthest_first_generalized =
      [this](const RelativePose3 &reference_pose,
             const ResourceManagerInterface::ManagedLocusHandle &lhs,
             const ResourceManagerInterface::ManagedLocusHandle &rhs) {
        // TODO(glpuga) update name of the lambda to account for this
        // additional check.
        // if difficulties are different, then prioritize placing easy parts
        // at the end of the list (from where we grab them)
        if (lhs.resource()->difficulty() != rhs.resource()->difficulty()) {
          return lhs.resource()->difficulty() > rhs.resource()->difficulty();
        }

        // if they are the same difficulty, the as a second criteria order
        // giving preference to those that are aligned with the reference

        const auto &lhs_pose = lhs.resource()->pose();
        const auto &rhs_pose = rhs.resource()->pose();
        // get all the poses in the same reference frame
        const auto target_lhs_pose = frame_transformer_->transformPoseToFrame(
            lhs_pose, reference_pose.frameId());
        const auto target_rhs_pose = frame_transformer_->transformPoseToFrame(
            rhs_pose, reference_pose.frameId());

        const auto ref_z = reference_pose.rotation().rotationMatrix().col(2);
        const auto lhs_z = lhs_pose.rotation().rotationMatrix().col(2);
        const auto rhs_z = rhs_pose.rotation().rotationMatrix().col(2);
        const auto lhs_proj = lhs_z.dot(ref_z);
        const auto rhs_proj = rhs_z.dot(ref_z);
        // if the projections have different signs, then order higher in
        // priority the part that is aligned with the reference
        if (lhs_proj * rhs_proj < 0) {
          return lhs_proj < 0;
        }

        // finally, given all else equal, order by distance, giving preference
        // to parts that are closer to the reference

        const auto lhs_distance = (reference_pose.position().vector() -
                                   target_lhs_pose.position().vector())
                                      .norm();
        const auto rhs_distance = (reference_pose.position().vector() -
                                   target_rhs_pose.position().vector())
                                      .norm();

        return lhs_distance > rhs_distance;
      };

  // next deal with unwanted pieces present in the container. Relocate them
  // somewhere else.
  if (unwanted_parts.size()) {
    std::vector<ResourceManagerInterface::ManagedLocusHandle> empty_loci;

    for (auto &part : unwanted_parts) {
      if (empty_loci.empty()) {
        // try to get a new list of empty loci
        empty_loci =
            resource_manager_->findEmptyLoci(free_radious_for_unwanted_pieces);

        // remove loci in agvs that are currently targeted by orders
        empty_loci.erase(std::remove_if(empty_loci.begin(), empty_loci.end(),
                                        active_agv_and_assemblies_filter),
                         empty_loci.end());

        if (empty_loci.empty()) {
          // there are no more empty spaces!
          WARNING("We ran out of empty loci in the environment!")
          break;
        }
      }

      auto sort_farthest_first =
          [this, sort_farthest_first_generalized,
           reference_pose = part.resource()->pose()](
              const ResourceManagerInterface::ManagedLocusHandle &lhs,
              const ResourceManagerInterface::ManagedLocusHandle &rhs) {
            return sort_farthest_first_generalized(reference_pose, lhs, rhs);
          };

      // sort by distance (larger distance first, to remove efficiently the
      // nearest spot from the list)
      std::sort(empty_loci.begin(), empty_loci.end(), sort_farthest_first);

      while (empty_loci.size()) {
        auto last_it = empty_loci.end() - 1;
        auto closest_empty_spot = std::move(*last_it);
        empty_loci.erase(last_it);

        std::set<WorkRegionId> work_regions;
        work_regions.emplace(resource_manager_->getWorkRegionId(part));
        work_regions.emplace(
            resource_manager_->getWorkRegionId(closest_empty_spot));

        auto robot_handle_opt =
            resource_manager_->getPickAndPlaceRobotHandle(work_regions);

        if (robot_handle_opt) {
          WARNING(
              "Creating a PickAndPlaceTask for {} to move an unwanted piece "
              "from {} to {}",
              robot_handle_opt->resource()->name(), part.resource()->pose(),
              closest_empty_spot.resource()->pose());
          output_actions.emplace_back(robot_task_factory_->getPickAndPlaceTask(
              std::move(part), std::move(closest_empty_spot),
              std::move(*robot_handle_opt)));
          break;
        }
      }
    }
  }

  int unavailable_part_count{0};

  // only move parts into the container if there's nothing moving out
  if (missing_parts.size()) {
    for (auto &[missing_part_locus, part_id] : missing_parts) {
      auto potential_sources =
          resource_manager_->findManagedLociByPartId(part_id);

      // remove parts in agvs that are currently targeted by orders
      potential_sources.erase(std::remove_if(potential_sources.begin(),
                                             potential_sources.end(),
                                             active_agv_and_assemblies_filter),
                              potential_sources.end());

      auto sort_farthest_first =
          [this, sort_farthest_first_generalized,
           reference_pose = missing_part_locus.resource()->pose()](
              const ResourceManagerInterface::ManagedLocusHandle &lhs,
              const ResourceManagerInterface::ManagedLocusHandle &rhs) {
            return sort_farthest_first_generalized(reference_pose, lhs, rhs);
          };

      // sort by distance (larger distance first, to remove efficiently the
      // nearest spot from the list)
      std::sort(potential_sources.begin(), potential_sources.end(),
                sort_farthest_first);

      // Choose the first one of the remaining lot, if any
      if (potential_sources.empty()) {
        ++unavailable_part_count;
      } else {
        // select the closest part
        auto &selected_source_part = *(potential_sources.end() - 1);

        const auto missing_part_region =
            resource_manager_->getWorkRegionId(missing_part_locus);
        const auto selected_part_region =
            resource_manager_->getWorkRegionId(selected_source_part);
        std::set<WorkRegionId> work_regions;
        work_regions.emplace(missing_part_region);
        work_regions.emplace(selected_part_region);
        auto robot_handle_opt =
            resource_manager_->getPickAndPlaceRobotHandle(work_regions);

        if (robot_handle_opt) {

          auto selected_source_part_orientation =
              selected_source_part.resource()
                  ->pose()
                  .rotation()
                  .rotationMatrix();
          auto missing_part_locus_orientation =
              missing_part_locus.resource()->pose().rotation().rotationMatrix();

          // dont flip parts that are not pumps
          auto [part_id, broken] = selected_source_part.resource()->model();
          (void)broken;

          // If we need to flip the part or if it comes from the conveyor belt,
          // find an empty space to leave the part as an intermediate stage
          // instead of carrying it to the destination.
          const bool conveyor_belt_piece =
              (selected_part_region == WorkRegionId::conveyor_belt);
          const bool part_needs_flipping =
              part_id.defined() && (part_id.type() == PartTypeId::pump) &&
              (selected_source_part_orientation.col(2).dot(
                   missing_part_locus_orientation.col(2)) <
               part_flipping_threshold_);

          // try to find a place where to flip it
          auto special_empty_loci = resource_manager_->findEmptyLoci(
              conveyor_belt_piece ? free_radious_for_unwanted_pieces
                                  : free_radious_for_flippable_pieces);

          // remove loci in agvs that are currently targeted by orders
          special_empty_loci.erase(
              std::remove_if(special_empty_loci.begin(),
                             special_empty_loci.end(),
                             active_agv_and_assemblies_filter),
              special_empty_loci.end());

          auto unreachable_empty_space =
              [this, work_regions](
                  const ResourceManagerInterface::ManagedLocusHandle &locus) {
                const auto locus_work_region =
                    resource_manager_->getWorkRegionId(locus);
                return work_regions.count(locus_work_region) == 0;
              };

          // remove loci not reachable by the robot
          special_empty_loci.erase(std::remove_if(special_empty_loci.begin(),
                                                  special_empty_loci.end(),
                                                  unreachable_empty_space),
                                   special_empty_loci.end());

          if (!special_empty_loci.empty() &&
              (conveyor_belt_piece || part_needs_flipping)) {
            auto sort_farthest_first =
                [this, sort_farthest_first_generalized,
                 reference_pose = missing_part_locus.resource()->pose()](
                    const ResourceManagerInterface::ManagedLocusHandle &lhs,
                    const ResourceManagerInterface::ManagedLocusHandle &rhs) {
                  return sort_farthest_first_generalized(reference_pose, lhs,
                                                         rhs);
                };

            // sort by distance to the target spot (larger distance first, to
            // remove efficiently the nearest spot from the list)
            std::sort(special_empty_loci.begin(), special_empty_loci.end(),
                      sort_farthest_first);

            auto last_it = special_empty_loci.end() - 1;
            auto closest_empty_spot = std::move(*last_it);

            if (conveyor_belt_piece) {
              WARNING("Creating a PickAndPlaceTask for {} to remove part "
                      "from conveyor belt at {} and into {}",
                      robot_handle_opt->resource()->name(),
                      selected_source_part.resource()->pose(),
                      closest_empty_spot.resource()->pose());
              output_actions.emplace_back(
                  robot_task_factory_->getPickAndPlaceTask(
                      std::move(selected_source_part),
                      std::move(closest_empty_spot),
                      std::move(*robot_handle_opt)));

            } else {
              WARNING("Creating a PickAndTwist for {} for a part at {}, "
                      "manipulating at {}",
                      robot_handle_opt->resource()->name(),
                      selected_source_part.resource()->pose(),
                      closest_empty_spot.resource()->pose());
              output_actions.emplace_back(
                  robot_task_factory_->getPickAndTwistPartTask(
                      std::move(selected_source_part),
                      std::move(closest_empty_spot),
                      std::move(*robot_handle_opt)));
            }
          } else {
            WARNING("Creating a PickAndPlaceTask for {} to provide a part from "
                    "{} and into {}",
                    robot_handle_opt->resource()->name(),
                    selected_source_part.resource()->pose(),
                    missing_part_locus.resource()->pose());
            output_actions.emplace_back(
                robot_task_factory_->getPickAndPlaceTask(
                    std::move(selected_source_part),
                    std::move(missing_part_locus),
                    std::move(*robot_handle_opt)));
          }
        }
      }
    }
  }

  bool shipping_done{false};

  // if there's nothing missing, then submit the shipping
  if ((broken_parts.size() == 0) && (unwanted_parts.size() == 0) &&
      (products.size() == parts_in_place.size() + unavailable_part_count)) {

    // release these pieces or we won't be able to capture the tray
    parts_in_place.clear();
    missing_parts.clear();

    auto tray_handle =
        resource_manager_->getSubmissionTray(target_container_name);

    if (tray_handle) {
      shipping_done = true;

      switch (shipment_class) {
      case ShipmentClass::Kitting:
        WARNING("Creating a SubmitKittingShipmentTask for {}",
                tray_handle->resource()->name());
        output_actions.emplace_back(
            robot_task_factory_->getSubmitKittingShipmentTask(
                std::move(tray_handle.value()), station_id, shipment_type));
        break;
      case ShipmentClass::Assembly:
        WARNING("Creating a SubmitAssemblyShipmentTask for {}",
                tray_handle->resource()->name());
        output_actions.emplace_back(
            robot_task_factory_->getSubmitAssemblyShipmentTask(
                std::move(tray_handle.value()), shipment_type));
        break;
      }
    }
  }

  return std::make_pair(std::move(output_actions), shipping_done);
}

std::pair<std::vector<RobotTaskInterface::Ptr>, bool>
OrderProcessingStrategy::processKittingShipment(
    const OrderId &order, const KittingShipment &shipment,
    const std::set<AgvId> &agvs_in_use,
    const std::set<StationId> &stations_in_use) const {
  return processUniversalShipment(
      ShipmentClass::Kitting, order, shipment.shipment_type, shipment.agv_id,
      shipment.station_id, shipment.products, agvs_in_use, stations_in_use);
}

std::pair<std::vector<RobotTaskInterface::Ptr>, bool>
OrderProcessingStrategy::processAssemblyShipment(
    const OrderId &order, const AssemblyShipment &shipment,
    const std::set<AgvId> &agvs_in_use,
    const std::set<StationId> &stations_in_use) const {
  return processUniversalShipment(
      ShipmentClass::Assembly, order, shipment.shipment_type, std::nullopt,
      shipment.station_id, shipment.products, agvs_in_use, stations_in_use);
}

} // namespace tijcore
