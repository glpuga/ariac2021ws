/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <algorithm>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

// tijcore
#include <tijcore/datatypes/QualifiedPartInfo.hpp>
#include <tijcore/resources/ResourceManager.hpp>
#include <tijcore/resources/SurfaceManager.hpp>
#include <tijcore/utils/CuboidVolume.hpp>
#include <tijlogger/logger.hpp>

namespace tijcore
{
namespace
{
constexpr std::chrono::milliseconds blocking_methods_sleep_interval_{ 200 };
}  // namespace

ResourceManager::ResourceManager(
    const Toolbox::SharedPtr& toolbox, std::vector<ModelContainerInterface::Ptr>&& model_containers,
    std::vector<PickAndPlaceRobotInterface::Ptr>&& pick_and_place_robots)
  : toolbox_{ toolbox }
{
  for (auto& container : model_containers)
  {
    if (model_containers_.count(container->name()))
    {
      throw std::runtime_error{ "Duplicated container name " + container->name() +
                                " in model containers" };
    }

    // Convert the unique_ptr into a shared ptr. I need to make a copy of the
    // name first, because the move operations has (obviously) side effects, so
    // the std::make_tuple() call may crash if it happens before reading the
    // first argument (that indeed happened in the first version of the code)
    auto container_name = container->name();
    model_containers_.emplace(
        std::make_pair(container_name, ModelContainerHandle{ std::move(container) }));
  }

  for (auto& robot : pick_and_place_robots)
  {
    pick_and_place_robots_.emplace_back(std::move(robot));
  }
}

std::vector<ResourceManagerInterface::ManagedLocusHandle>
ResourceManager::findVacantLociCandidates(const double free_radius)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  clearNonAllocatedEmptyLoci();

  std::map<std::string, SurfaceManager> surface_occupancy_maps;
  for (const auto& model_container_entry : model_containers_)
  {
    // Only consider containers that are not currently allocated to some task,
    // and not disabled
    auto& model_container_handle = model_container_entry.second;
    if (!model_container_handle.allocated() &&
        model_container_handle.resource()->enabled())  // TODO(glpuga) Do I still need to keept the
                                                       // "allocated" for surfaces?
    {
      surface_occupancy_maps.emplace(std::make_pair(
          model_container_entry.first, buildContainerSurfaceManager(model_container_entry.first)));
    }
  }

  std::vector<ResourceManagerInterface::ManagedLocusHandle> found_loci;

  // finally, try to find at least one candidate from each available
  // model surface
  for (const auto& surface_map_entry : surface_occupancy_maps)
  {
    auto& [container_name, container_surface] = surface_map_entry;

    auto region_opt = container_surface.findFreeRegion(free_radius);
    if (region_opt)
    {
      auto parent_pose = model_containers_.at(container_name).resource()->pose();
      auto [x, y] = region_opt.value();

      auto parent_ref_frame =
          model_containers_.at(container_name).resource()->containerReferenceFrameId();

      tijmath::RelativePose3 pose{ parent_ref_frame, tijmath::Position::fromVector(x, y, 0),
                                   tijmath::Rotation::Identity };

      // record it locally and add it to the list of free loci
      {
        ManagedLocusHandle new_handle{ std::make_shared<ManagedLocus>(
            ManagedLocus::CreateEmptyLocus(container_name, pose)) };
        known_loci_resource_state_.emplace_back(new_handle);
        found_loci.emplace_back(std::move(new_handle));
      }
    }
    else
    {
      DEBUG("No free region in this surface");
    }
  }

  return found_loci;
}

std::vector<ResourceManagerInterface::ManagedLocusHandle>
ResourceManager::getPartSourceListByType(const PartId& part_id)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  clearNonAllocatedEmptyLoci();

  auto filter = [this, &part_id](const ManagedLocusHandle& handle) {
    if (handle.allocated() || !handle.resource()->isLocusWithPart())
    {
      return false;
    }

    // TODO(glpuga) to allow or not to allow to return models in allocated
    // containers? if I dont', it artificially makes think that there are not
    // more sources of a given part id, and can cause shipping to be dispatched
    // empty. If I do, robots may spend time waiting for parts because another
    // robot is using the airspace.
    // if the parent is allocated or disabled, do not consider it
    // TODO(glpuga) add test case for the allocated case
    auto& parent_handle = model_containers_.at(handle.resource()->parentName());
    if (/* parent_handle.allocated() || */ !parent_handle.resource()->enabled())
    {
      return false;
    }
    return part_id == handle.resource()->qualifiedPartInfo().part_type;
  };

  std::vector<ResourceManagerInterface::ManagedLocusHandle> output;
  std::copy_if(known_loci_resource_state_.begin(), known_loci_resource_state_.end(),
               std::back_inserter(output), filter);
  return output;
}

std::vector<ResourceManagerInterface::ManagedLocusHandle>
ResourceManager::getMovableTraySourceListByType(const MovableTrayId& movable_tray_type)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  clearNonAllocatedEmptyLoci();

  auto filter = [this, &movable_tray_type](const ManagedLocusHandle& handle) {
    if (handle.allocated() || !handle.resource()->isLocusWithMovableTray())

    {
      return false;
    }

    // TODO(glpuga) to allow or not to allow to return models in allocated
    // containers? if I dont', it artificially makes think that there are not
    // more sources of a given part id, and can cause shipping to be dispatched
    // empty. If I do, robots may spend time waiting for parts because another
    // robot is using the airspace.
    // if the parent is allocated or disabled, do not consider it
    // TODO(glpuga) add test case for the allocated case
    auto& parent_handle = model_containers_.at(handle.resource()->parentName());
    if (/* parent_handle.allocated() || */ !parent_handle.resource()->enabled())
    {
      return false;
    }
    return movable_tray_type == handle.resource()->qualifiedMovableTrayInfo().tray_type;
  };

  std::vector<ResourceManagerInterface::ManagedLocusHandle> output;
  std::copy_if(known_loci_resource_state_.begin(), known_loci_resource_state_.end(),
               std::back_inserter(output), filter);
  return output;
}

std::vector<ResourceManagerInterface::ManagedLocusHandle>
ResourceManager::findSiblingLociByCommonParent(const std::string& parent_name)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  clearNonAllocatedEmptyLoci();

  // TODO(glpuga) add test for these cases
  // if the parent is allocated or disabled, return no candidates
  auto& parent_handle = model_containers_.at(parent_name);
  if (parent_handle.allocated() || !parent_handle.resource()->enabled())
  {
    return {};
  }

  auto filter = [this, &parent_name](const ManagedLocusHandle& handle) {
    if (handle.allocated() || !handle.resource()->isLocusWithPart())
    {
      return false;
    }
    return parent_name == handle.resource()->parentName();
  };

  std::vector<ResourceManagerInterface::ManagedLocusHandle> output;
  std::copy_if(known_loci_resource_state_.begin(), known_loci_resource_state_.end(),
               std::back_inserter(output), filter);
  return output;
}

std::optional<ResourceManagerInterface::PickAndPlaceRobotHandle>
ResourceManager::getPickAndPlaceRobotHandle(const std::vector<tijmath::RelativePose3>& waypoints)
{
  std::lock_guard<std::mutex> lock{ mutex_ };

  std::optional<PickAndPlaceRobotHandle> current_selection_opt;

  for (auto& pick_and_place_robot_handle : pick_and_place_robots_)
  {
    // discard robots that have already been allocated to a task
    if (pick_and_place_robot_handle.allocated())
    {
      continue;
    }

    // discard robots that that have been disabled
    if (!pick_and_place_robot_handle.resource()->enabled())
    {
      continue;
    }

    // discard robots that are unable to reach one or more waypoints
    auto reachability_test =
        [&pick_and_place_robot_handle](const tijmath::RelativePose3& waypoint) {
          return pick_and_place_robot_handle.resource()->canReach(waypoint);
        };
    if (!std::all_of(waypoints.begin(), waypoints.end(), reachability_test))
    {
      continue;
    }

    current_selection_opt = pick_and_place_robot_handle;
    break;
  }

  return current_selection_opt;
}

std::optional<ResourceManagerInterface::ManagedLocusHandle>
ResourceManager::getLocusAtPose(const tijmath::RelativePose3& pose)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  clearNonAllocatedEmptyLoci();

  // check if the pose matches that of a known managed locus
  auto shortest_distance_to_pose = [this, &reference_pose =
                                              pose](const ManagedLocusHandle& lhs_locus_handle,
                                                    const ManagedLocusHandle& rhs_locus_handle) {
    auto& lhs_locus = *lhs_locus_handle.resource();
    auto& rhs_locus = *rhs_locus_handle.resource();

    // get all poses in the same frame
    auto lhs_pose = transformPoseToContainerLocalPose(lhs_locus.pose(), reference_pose.frameId());
    auto rhs_pose = transformPoseToContainerLocalPose(rhs_locus.pose(), reference_pose.frameId());

    // ignore differences in height
    lhs_pose.position().vector().z() = reference_pose.position().vector().z();
    rhs_pose.position().vector().z() = reference_pose.position().vector().z();

    const auto lhs_distance =
        (lhs_pose.position().vector() - reference_pose.position().vector()).norm();
    const auto rhs_distance =
        (rhs_pose.position().vector() - reference_pose.position().vector()).norm();

    return lhs_distance < rhs_distance;
  };

  if (known_loci_resource_state_.size())
  {
    // determine the model that's closest to the requested pose
    const auto closest_part =
        std::min_element(known_loci_resource_state_.begin(), known_loci_resource_state_.end(),
                         shortest_distance_to_pose);

    // get both poses in the same reference frame
    auto reframed_closest_part_pose =
        transformPoseToContainerLocalPose(closest_part->resource()->pose(), pose.frameId());

    // ignore differences in height
    reframed_closest_part_pose.position().vector().z() = pose.position().vector().z();

    // they get compared without rotation, because we may not know the
    // rotation value
    if (tijmath::Position::samePosition(reframed_closest_part_pose.position(), pose.position(),
                                        locus_identity_position_tolerance_))
    {
      return *closest_part;
    }
  }

  // none of the existing parts could be matched to the requeste pose.
  // Create a new locus for an empty space.
  auto parent_container = getPoseParentContainerPtr(pose);

  if (!parent_container)
  {
    // it does not belong to any container!
    ERROR(
        "Requested a managed locus in a location that does not belong to any "
        "container!");
    return std::nullopt;
  }

  if (!parent_container->resource()->enabled())
  {
    // the container is disabled, can't provide a locus from here
    ERROR("Requested a managed locus from a disabled container!");
    return std::nullopt;
  }

  auto parent_name = parent_container->resource()->name();
  auto parent_frame_id = parent_container->resource()->containerReferenceFrameId();

  // check if we have surface real state to allocate this
  auto surface_manager = buildContainerSurfaceManager(parent_name);

  auto [x, y, r] = poseToOccupancy(pose, parent_frame_id);

  if (!surface_manager.regionIsWithinSurface(x, y, r) ||
      !surface_manager.regionIsAvailable(x, y, r))
  {
    ERROR(
        "Could not create an empty space, there's no free space in {}  "
        "at the requested location",
        parent_container->resource()->name());
    return std::nullopt;
  }

  // we're ok, create a handle for this and return it.
  auto target_locus_handle = ResourceManagerInterface::ManagedLocusHandle(
      std::make_shared<ManagedLocus>(ManagedLocus::CreateEmptyLocus(parent_name, pose)));

  known_loci_resource_state_.emplace_back(target_locus_handle);

  return std::optional<ManagedLocusHandle>{ std::move(target_locus_handle) };
}

SurfaceManager ResourceManager::volumeToSurface(const CuboidVolume& v) const
{
  const auto lrb = v.lowerRightBackCorner();
  const auto ulf = v.upperLeftFrontCorner();
  auto x0 = lrb.vector().x();
  auto y0 = lrb.vector().y();
  auto width = ulf.vector().x() - lrb.vector().x();
  auto height = ulf.vector().y() - lrb.vector().y();
  return SurfaceManager(x0, y0, width, height);
}

SurfaceManager ResourceManager::buildContainerSurfaceManager(const std::string& parent_name) const
{
  auto handle_ptr = model_containers_.at(parent_name).resource();
  auto parent_pose = handle_ptr->pose();

  auto surface_manager = volumeToSurface(handle_ptr->containerVolume());

  for (const auto& model_locus : known_loci_resource_state_)
  {
    // TODO(glpuga) This is hacky and potentially dangerous
    // only consider loci for part in the part layer
    if (getComponentLayer(*model_locus.resource()) != ComponentLayer::PartLayer)
    {
      continue;
    }

    auto [x, y, r] =
        poseToOccupancy(model_locus.resource()->pose(), handle_ptr->containerReferenceFrameId());
    surface_manager.markAsOccupied(x, y, r);
  }

  return surface_manager;
}

std::vector<ManagedLocus> ResourceManager::createObservedLociListFromObservedModels(
    const std::vector<ObservedItem>& observed_models) const
{
  std::vector<ManagedLocus> new_model_loci;

  // create a list of model locus that can be matched to a container
  for (const auto& observed_item_entry : observed_models)
  {
    auto parent_container = getPoseParentContainerPtr(observed_item_entry.pose);

    if (!parent_container)
    {
      WARNING(
          "A model at {} appears in sensor input but could not be matched "
          "to a known surface",
          transformPoseToContainerLocalPose(observed_item_entry.pose, "world"));
      continue;
    }

    // TODO(glpuga): do we still need surface relative poses?
    // this transformation makes all objects relative to the surface. This is
    // the same as the relative to the container, except for the conveyor belt.
    const auto parent_surface_frame_id = parent_container->resource()->surfaceReferenceFrameId();
    auto surface_local_frame_pose = tijmath::RelativePose3(
        parent_surface_frame_id,
        transformPoseToContainerLocalPose(observed_item_entry.pose, parent_surface_frame_id));

    const auto parent_name = parent_container->resource()->name();

    if (observed_item_entry.item.is<QualifiedPartInfo>())
    {
      const auto& observed_part = observed_item_entry.item.as<QualifiedPartInfo>();
      auto model_locus = ManagedLocus::CreatePartLocus(  //
          parent_name,                                   //
          surface_local_frame_pose,                      //
          observed_part.part_type,                       //
          observed_part.part_is_broken);                 //
      new_model_loci.emplace_back(std::move(model_locus));
    }
    else if (observed_item_entry.item.is<QualifiedMovableTrayInfo>())
    {
      const auto& observed_movable_tray = observed_item_entry.item.as<QualifiedMovableTrayInfo>();
      auto model_locus = ManagedLocus::CreateMovableTrayLocus(  //
          parent_name,                                          //
          surface_local_frame_pose,                             //
          observed_movable_tray.tray_type);                     //
      new_model_loci.emplace_back(std::move(model_locus));
    }
    else
    {
      WARNING(
          "An item at the resource manager input is of a type not known to the resource maanger "
          "and will be ignored.");
      continue;
    }
  }
  return new_model_loci;
}

void ResourceManager::processInputSensorData(const std::vector<ObservedItem>& observed_models)
{
  std::lock_guard<std::mutex> lock{ mutex_ };

  if (!observed_models.size())
  {
    WARNING("No models visible on any sensor! Blackout?");
    return;
  }

  auto new_model_loci = createObservedLociListFromObservedModels(observed_models);

  // This will be a list of the elements that we want to keep in the state vector
  std::set<tijutils::UniqueId> uids_of_the_loci_to_keep;

  //
  // move over known allocated loci. We won't be updating those from input information
  for (auto& known_model_locus_handle : known_loci_resource_state_)
  {
    // allocated objects need to be carried over, both empty and non-empty ones
    if (known_model_locus_handle.allocated())
    {
      uids_of_the_loci_to_keep.insert(known_model_locus_handle.resource()->uniqueId());
    }
  }

  //
  // find matches between observed camera models and known model loci. Add new
  // ones, and improve information on the others.
  for (const auto& new_model_locus : new_model_loci)
  {
    bool found{ false };

    for (auto& known_model_locus_handle : known_loci_resource_state_)
    {
      auto& known_model_locus = *known_model_locus_handle.resource();

      // make sure both poses are known in the same frame
      auto known_model_locus_pose = known_model_locus.pose();
      auto new_model_locus_pose = transformPoseToContainerLocalPose(
          new_model_locus.pose(), known_model_locus_pose.frameId());

      // ignore differences in height
      new_model_locus_pose.position().vector().z() = known_model_locus_pose.position().vector().z();

      // they get compared without rotation, because we may not know the
      // rotation value
      if (tijmath::Position::samePosition(new_model_locus_pose.position(),
                                          known_model_locus_pose.position(),
                                          locus_identity_position_tolerance_))
      {
        // if they belong in different layers, don't bother going any further
        if (getComponentLayer(new_model_locus) != getComponentLayer(known_model_locus))
        {
          continue;
        }

        // don't update parts that are involved in active tasks
        if (known_model_locus_handle.allocated())
        {
          found = true;
          break;
        }

        // both are one an the same (based on position), so we use the new
        // information to improve what we know already about the locus
        if (known_model_locus.isEmptyLocus())
        {
          // if we knew nothing, then any new that we got information is valid
          known_model_locus = new_model_locus;
        }
        else if (known_model_locus.isLocusWithPart() && new_model_locus.isLocusWithPart())
        {
          known_model_locus =
              mergeOldAndNewPartLocusInformation(known_model_locus, new_model_locus);
          uids_of_the_loci_to_keep.emplace(known_model_locus.uniqueId());
        }
        else if (known_model_locus.isLocusWithMovableTray() &&
                 new_model_locus.isLocusWithMovableTray())
        {
          known_model_locus =
              mergeOldAndNewMovableTrayLocusInformation(known_model_locus, new_model_locus);
          uids_of_the_loci_to_keep.emplace(known_model_locus.uniqueId());
        }

        // one way or the other, we already had information on this locus that
        // we merged with new data, so stop looking for it.
        found = true;
        break;
      }
    }

    // if we didn't find a match, then we need to add a new locus
    if (!found)
    {
      auto it =
          known_loci_resource_state_.emplace_back(std::make_shared<ManagedLocus>(new_model_locus));
      uids_of_the_loci_to_keep.emplace(it.resource()->uniqueId());
    }
  }

  // This filter will remove any locus in known_loci_resource_state_ that is not in
  // uids_of_the_loci_to_keep
  auto filter = [&uids_of_the_loci_to_keep](const ManagedLocusHandle& locus) {
    return uids_of_the_loci_to_keep.count(locus.resource()->uniqueId()) == 0;
  };

  // remove from the known_loci_resource_state_ list all those locus that have not been moved
  // and that are not allocated to new tasks. Those will be empty locus and
  // pieces that are not currently visible in the camera.
  known_loci_resource_state_.erase(
      std::remove_if(known_loci_resource_state_.begin(), known_loci_resource_state_.end(), filter),
      known_loci_resource_state_.end());

  //
  // After all of the above known_loci_resource_state_ contains:
  // - Known models located in allocated containers.
  // - New models in non-allocated containers.
  // - known models in non-allocated containers visible in camera.
  // this leaves behind
  // - known models not visible in camera in non-allocated containers
  // - new models in allocated containers
}

const ResourceManager::ModelContainerHandle*
ResourceManager::getPoseParentContainerPtr(const tijmath::RelativePose3& pose) const
{
  for (const auto& container : model_containers_)
  {
    if (poseIsOnPartContainer(pose, container.second))
    {
      return &container.second;
    }
  }
  return nullptr;
}

ResourceManager::ComponentLayer ResourceManager::getComponentLayer(const ManagedLocus& locus) const
{
  if (locus.isLocusWithMovableTray())
  {
    return ComponentLayer::MovableTrayLayer;
  }
  // else

  return ComponentLayer::PartLayer;
}

ManagedLocus ResourceManager::mergeOldAndNewPartLocusInformation(
    const ManagedLocus& known_locus_data, const ManagedLocus& new_locus_data) const
{
  auto known_part_id = known_locus_data.qualifiedPartInfo().part_type;
  auto known_broken = known_locus_data.qualifiedPartInfo().part_is_broken;

  const auto new_part_id = new_locus_data.qualifiedPartInfo().part_type;
  const auto new_broken = new_locus_data.qualifiedPartInfo().part_is_broken;

  // unless the new information is less specific than what was already knonw, update
  if (new_part_id != PartId::UnkownPartId)
  {
    known_part_id = new_part_id;
  }

  // TODO(glpuga) Once broken, a model cannot be considered non-broken
  // again this should be fine for this competition, but not in
  // general. Now knowing if something is broken or not should be an
  // option.
  known_broken = known_broken || new_broken;

  // we always update the pose information with the new information.
  // We know that position is the same, but rotation might have
  // changed. This will also update changes within tolerance.
  auto known_pose = new_locus_data.pose();

  // Create the updated locus with the new information
  // and add it to the list of loci that are part of the updated state
  // take care to keep the same uniqueId
  auto updated_locus = ManagedLocus::CreatePartLocus(new_locus_data.parentName(), known_pose,
                                                     known_part_id, known_broken);
  updated_locus.resetUniqueId(known_locus_data.uniqueId());
  return updated_locus;
}

ManagedLocus ResourceManager::mergeOldAndNewMovableTrayLocusInformation(
    const ManagedLocus& known_locus_data, const ManagedLocus& new_locus_data) const
{
  // nothing really to update here. We just overwrite old information with the new one.
  auto updated_locus_data = new_locus_data;
  updated_locus_data.resetUniqueId(known_locus_data.uniqueId());
  return updated_locus_data;
}

void ResourceManager::clearEmptyLoci()
{
  auto filter = [](const ManagedLocusHandle& locus) {
    // remove it if it's not currently in use and it represents
    // an empty locus
    return !locus.allocated() && locus.resource()->isEmptyLocus();
  };
  known_loci_resource_state_.erase(
      std::remove_if(known_loci_resource_state_.begin(), known_loci_resource_state_.end(), filter),
      known_loci_resource_state_.end());
}

tijmath::Pose3 ResourceManager::transformPoseToContainerLocalPose(
    const tijmath::RelativePose3& pose, const std::string& container_frame_id) const
{
  auto frame_transformer = toolbox_->getFrameTransformer();
  auto transformed_pose = frame_transformer->transformPoseToFrame(pose, container_frame_id);
  return transformed_pose.pose();
}

std::tuple<double, double, double> ResourceManager::poseToOccupancy(
    const tijmath::RelativePose3& relative_pose, const std::string& container_frame_id) const
{
  tijmath::Pose3 local_pose = transformPoseToContainerLocalPose(relative_pose, container_frame_id);
  auto local_vector = local_pose.position().vector();
  return std::make_tuple(local_vector.x(), local_vector.y(), default_occupancy_radius_);
}

bool ResourceManager::poseIsOnPartContainer(const tijmath::RelativePose3& pose,
                                            const ModelContainerHandle& container) const
{
  const auto& container_volume = container.resource()->containerVolume();
  const auto& local_frame_id = container.resource()->containerReferenceFrameId();
  const auto local_pose = transformPoseToContainerLocalPose(pose, local_frame_id);
  return container_volume.contains(local_pose.position());
}

void ResourceManager::clearNonAllocatedEmptyLoci()
{
  auto filter = [](const ManagedLocusHandle& locus) {
    return !locus.allocated() && locus.resource()->isEmptyLocus();
  };

  known_loci_resource_state_.erase(
      std::remove_if(known_loci_resource_state_.begin(), known_loci_resource_state_.end(), filter),
      known_loci_resource_state_.end());
}

void ResourceManager::logCurrentResourceManagerState()
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  clearNonAllocatedEmptyLoci();

  WARNING("Known world state ({} loci)", known_loci_resource_state_.size());
  for (auto& known_model_locus_handle : known_loci_resource_state_)
  {
    auto& known_model_locus = *known_model_locus_handle.resource();

    // Convert to the frame of the parent
    auto known_model_locus_pose = known_model_locus.pose();
    auto parent_frame = model_containers_.at(known_model_locus.parentName())
                            .resource()
                            ->containerReferenceFrameId();
    auto pose_in_parent = tijmath::RelativePose3{
      parent_frame, transformPoseToContainerLocalPose(known_model_locus_pose, parent_frame)
    };
    if (known_model_locus.isEmptyLocus())
    {
      INFO(" - empty at {} (allocated: {}, uid: {})", pose_in_parent,
           known_model_locus_handle.allocated(), known_model_locus.uniqueId());
    }
    else if (known_model_locus.isLocusWithPart())
    {
      const auto qualified_part_info = known_model_locus.qualifiedPartInfo();
      INFO(" - {} at {} (broken: {} , allocated: {}, uid: {})",
           qualified_part_info.part_type.codedString(), pose_in_parent,
           qualified_part_info.part_is_broken, known_model_locus_handle.allocated(),
           known_model_locus.uniqueId());
    }
    else if (known_model_locus.isLocusWithMovableTray())
    {
      const auto qualified_movable_tray_info = known_model_locus.qualifiedMovableTrayInfo();
      INFO(" - {} at {} (allocated: {}, uid: {})", qualified_movable_tray_info.tray_type,
           pose_in_parent, known_model_locus_handle.allocated(), known_model_locus.uniqueId());
    }
  }
  INFO("---");
}

}  // namespace tijcore
