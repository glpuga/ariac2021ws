/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library
#include <algorithm>
#include <iostream>
#include <iterator>
#include <map>
#include <mutex>
#include <string>

// tijcore
#include <logger/logger.hpp>
#include <tijcore/agents/SurfaceManager.hpp>
#include <tijcore/math/CuboidVolume.hpp>
#include <tijcore/perception/ModelTraySharedAccessSpaceDescription.hpp>
#include <tijcore/perception/ResourceManager.hpp>

namespace tijcore {

namespace {
constexpr std::chrono::milliseconds blocking_methods_sleep_interval_{200};

constexpr int32_t conveyor_belt_start_difficulty_ = 3;
} // namespace

ResourceManager::ResourceManager(
    const Toolbox::SharedPtr &toolbox,
    const std::vector<ModelTraySharedAccessSpaceDescription>
        &shared_workspace_descriptors,
    std::vector<ModelContainerInterface::Ptr> &&model_containers,
    std::vector<PickAndPlaceRobotInterface::Ptr> &&pick_and_place_robots)
    : shared_workspace_descriptors_{shared_workspace_descriptors},
      toolbox_{toolbox} {

  for (auto &shared_workspace : shared_workspace_descriptors) {
    if (shared_working_spaces_.count(shared_workspace.id)) {
      WARNING("Duplicated id {} in shared_workspace_descriptors",
              shared_workspace.id);
    }

    shared_working_spaces_.emplace(
        std::make_pair(shared_workspace.id,
                       SharedWorkspaceHandle{std::make_unique<std::string>(
                           shared_workspace.id)}));
  }

  for (auto &container : model_containers) {
    if (model_containers_.count(container->name())) {
      throw std::runtime_error{"Duplicated container name " +
                               container->name() + " in model containers"};
    }

    if (!shared_working_spaces_.count(container->exclusionZoneId())) {
      throw std::runtime_error{
          "The container " + container->name() +
          " has a shared workspace id not defined in the descriptors vector (" +
          container->exclusionZoneId() + ")"};
    }

    // Convert the unique_ptr into a shared ptr. I need to make a copy of the
    // name first, because the move operations has (obviously) side effects, so
    // the std::make_tuple() call may crash if it happens before reading the
    // first argument (that indeed happened in the first version of the code)
    auto container_name = container->name();
    model_containers_.emplace(std::make_pair(
        container_name, ModelContainerHandle{std::move(container)}));
  }

  for (auto &robot : pick_and_place_robots) {
    pick_and_place_robots_.emplace_back(std::move(robot));
  }
}

std::vector<ResourceManagerInterface::ManagedLocusHandle>
ResourceManager::findEmptyLoci() {
  std::lock_guard<std::mutex> lock{mutex_};
  clearNonAllocatedEmptyLoci();

  std::map<std::string, SurfaceManager> surface_occupancy_maps;
  for (const auto &model_container_entry : model_containers_) {
    // Only consider containers that are not currently allocated to some task,
    // and not disabled
    auto &model_container_handle = model_container_entry.second;
    if (!model_container_handle.allocated() &&
        model_container_handle.resource()->enabled()) {
      surface_occupancy_maps.emplace(std::make_pair(
          model_container_entry.first,
          buildContainerSurfaceManager(model_container_entry.first)));
    }
  }

  std::vector<ResourceManagerInterface::ManagedLocusHandle> found_loci;

  // finally, try to find at least one candidate from each available
  // model surface
  for (const auto &surface_map_entry : surface_occupancy_maps) {
    auto &[container_name, container_surface] = surface_map_entry;

    auto region_opt =
        container_surface.findFreeRegion(default_occupancy_radius_);
    if (region_opt) {
      auto parent_pose =
          model_containers_.at(container_name).resource()->pose();
      auto [x, y] = region_opt.value();

      auto parent_ref_frame = model_containers_.at(container_name)
                                  .resource()
                                  ->containerReferenceFrameId();

      RelativePose3 pose{parent_ref_frame, Position::fromVector(x, y, 0),
                         Rotation::Identity};

      // record it locally and add it to the list of free loci
      {
        ManagedLocusHandle new_handle{std::make_shared<ManagedLocus>(
            ManagedLocus::CreateEmptySpace(container_name, pose))};
        model_loci_.emplace_back(new_handle);
        found_loci.emplace_back(std::move(new_handle));
      }

    } else {
      DEBUG("No free region in this surface");
    }
  }

  return found_loci;
}

std::vector<ResourceManagerInterface::ManagedLocusHandle>
ResourceManager::findManagedLociByPartId(const PartId &part_id) {
  std::lock_guard<std::mutex> lock{mutex_};
  clearNonAllocatedEmptyLoci();

  auto filter = [this, &part_id](const ManagedLocusHandle &handle) {
    if (handle.allocated() || !handle.resource()->isModel()) {
      return false;
    }

    // TODO(glpuga) to allow or not to allow to return models in allocated
    // containers? if I dont', it artificially makes think that there are not
    // more sources of a given part id, and can cause shipping to be dispatched
    // empty. If I do, robots may spend time waiting for parts because another
    // robot is using the airspace.
    // if the parent is allocated or disabled, do not consider it
    // TODO(glpuga) add test case for the allocated case
    auto &parent_handle = model_containers_.at(handle.resource()->parentName());
    if (/* parent_handle.allocated() || */ !parent_handle.resource()
            ->enabled()) {
      return false;
    }
    auto [known_model_part_id, broken] = handle.resource()->model();
    (void)broken; // avoids unused variable warning
    return part_id == known_model_part_id;
  };

  std::vector<ResourceManagerInterface::ManagedLocusHandle> output;
  std::copy_if(model_loci_.begin(), model_loci_.end(),
               std::back_inserter(output), filter);
  return output;
}

std::vector<ResourceManagerInterface::ManagedLocusHandle>
ResourceManager::findManagedLociByParent(const std::string &parent_name) {
  std::lock_guard<std::mutex> lock{mutex_};
  clearNonAllocatedEmptyLoci();

  // TODO(glpuga) add test for these cases
  // if the parent is allocated or disabled, return no candidates
  auto &parent_handle = model_containers_.at(parent_name);
  if (parent_handle.allocated() || !parent_handle.resource()->enabled()) {
    return {};
  }

  auto filter = [this, &parent_name](const ManagedLocusHandle &handle) {
    if (handle.allocated()) {
      return false;
    }
    return parent_name == handle.resource()->parentName();
  };

  std::vector<ResourceManagerInterface::ManagedLocusHandle> output;
  std::copy_if(model_loci_.begin(), model_loci_.end(),
               std::back_inserter(output), filter);
  return output;
}

std::optional<ResourceManagerInterface::PickAndPlaceRobotHandle>
ResourceManager::getPickAndPlaceRobotHandle(
    const std::set<WorkRegionId> &regions) {
  std::lock_guard<std::mutex> lock{mutex_};

  // Determine the fitness index of the robot to a task. nullopt means
  // unable to perform; if not nullopt, the smaller the number the better.
  auto calculate_robot_to_task_fitness_to_task =
      [](const std::set<WorkRegionId> &task_regions,
         const std::set<WorkRegionId> &robot_supported_regions)
      -> std::optional<size_t> {
    bool can_do{true};
    for (const auto &region : task_regions) {
      can_do = can_do && (robot_supported_regions.count(region) > 0);
    }
    if (!can_do) {
      // the robot does not have reach the minimum set of regions required
      // by the task
      return std::nullopt;
    }
    // the robot can do the task, so the fitness now measures how tightly the
    // regions supported by the robot fit the ones required by the task
    return robot_supported_regions.size() - task_regions.size();
  };

  std::optional<PickAndPlaceRobotHandle> current_selection_opt;
  size_t current_selection_fitness_level{0};

  for (auto &pap_robot_handle : pick_and_place_robots_) {

    // discard robots that have already been allocated to a task
    if (pap_robot_handle.allocated()) {
      continue;
    }

    // discard robots that that have been disabled
    if (!pap_robot_handle.resource()->enabled()) {
      continue;
    }

    auto robot_supported_regions =
        pap_robot_handle.resource()->supportedRegions();

    auto fitness_level_opt = calculate_robot_to_task_fitness_to_task(
        regions, robot_supported_regions);

    if (fitness_level_opt &&
        (!current_selection_opt ||
         (current_selection_fitness_level > fitness_level_opt.value()))) {
      current_selection_opt = pap_robot_handle;
      current_selection_fitness_level = fitness_level_opt.value();
    }
  }
  return current_selection_opt;
}

std::optional<ResourceManagerInterface::ExclusionZoneHandle>
ResourceManager::getModelTrayExclusionZoneHandle(
    const std::string &model_tray_name,
    const std::optional<std::string> &additional_model_tray_name_opt,
    const std::chrono::milliseconds &timeout) {
  std::unique_lock<std::mutex> lock{mutex_};

  if (!model_containers_.count(model_tray_name)) {
    // what container did you say?
    ERROR("Could not provide access to container airspace because the name "
          "does not match any known container");
    return std::nullopt;
  }

  // notice the hackish way to avoid having to check multiple times if a second
  // tray name was provided
  std::string additional_model_tray_name;
  if (additional_model_tray_name_opt) {
    additional_model_tray_name = additional_model_tray_name_opt.value();
  } else {
    // if none was given, we'll check twice for the same
    additional_model_tray_name = model_tray_name;
  }

  if (!model_containers_.count(additional_model_tray_name)) {
    // what container did you say?
    ERROR("An additional container name was provided, but the name does not "
          "match any known container");
    return std::nullopt;
  }

  const auto &model_tray_handle = model_containers_.at(model_tray_name);
  const auto &additional_model_tray_handle =
      model_containers_.at(model_tray_name);

  const auto &model_tray_exclusion_zone_handle = shared_working_spaces_.at(
      model_tray_handle.resource()->exclusionZoneId());

  const auto start_time = std::chrono::system_clock::now();

  const auto logging_names =
      additional_model_tray_name_opt
          ? model_tray_name + " and " + additional_model_tray_name
          : model_tray_name;

  const auto initial_sensor_update_count = sensor_update_count_;

  while ((initial_sensor_update_count == sensor_update_count_) ||
         model_tray_handle.allocated() ||
         additional_model_tray_handle.allocated() ||
         model_tray_exclusion_zone_handle.allocated()) {
    const auto current_time = std::chrono::system_clock::now();
    if (current_time - start_time > timeout) {
      WARNING("Timed out while waiting to get access the exclusion zone of {}",
              logging_names);
      return std::nullopt;
    }

    {
      lock.unlock();
      std::this_thread::sleep_for(blocking_methods_sleep_interval_);
      lock.lock();
    }
  }

  if (!model_tray_handle.resource()->enabled() ||
      !additional_model_tray_handle.resource()->enabled()) {
    WARNING("Failed to get a handle to the exclusion zone of {} "
            "because at least one of the trays is disabled",
            logging_names);
    return std::nullopt;
  }

  std::vector<ModelContainerHandle> locked_containers;
  locked_containers.push_back(model_tray_handle);
  if (additional_model_tray_name_opt) {
    locked_containers.push_back(model_tray_handle);
  }

  return ExclusionZoneHandle(
      std::make_shared<ModelTraySharedAccessSpaceResource>(
          std::move(locked_containers), model_tray_exclusion_zone_handle));
}

std::optional<ResourceManagerInterface::SubmissionTrayHandle>
ResourceManager::getSubmissionTray(const std::string &model_container_name) {
  std::lock_guard<std::mutex> lock{mutex_};
  clearNonAllocatedEmptyLoci();

  if (!model_containers_.count(model_container_name)) {
    // what container did you say?
    ERROR("Could not provide access to container tray because the name does "
          "not match any known container");
    return std::nullopt;
  }

  const auto &container_handle_ref = model_containers_.at(model_container_name);

  if (container_handle_ref.allocated()) {
    // TODO(glpuga) test this case I just added
    // the working space around he container is in use, sorry.
    return std::nullopt;
  }

  if (!container_handle_ref.resource()->enabled()) {
    // TODO(glpuga) should we be able to allocate trays in disabled containers?
    // how do we enable them again?
    return std::nullopt;
  }

  // the block can only be granted if there are no allocated handles on it.
  bool container_has_allocated_handles{false};
  auto handle_checker =
      [&container_has_allocated_handles,
       &model_container_name](const ManagedLocusHandle &handle) {
        const auto handle_in_container =
            (model_container_name == handle.resource()->parentName());
        const auto handle_is_allocated = handle.allocated();
        container_has_allocated_handles =
            container_has_allocated_handles ||
            (handle_is_allocated && handle_in_container);
      };

  std::for_each(model_loci_.begin(), model_loci_.end(), handle_checker);

  if (container_has_allocated_handles) {
    // can't allocate the resource with active model handles in it
    return std::nullopt;
  }

  return SubmissionTrayHandle(
      std::make_shared<SubmissionTrayAdapter>(container_handle_ref));
}

std::string ResourceManager::getContainerFrameId(
    const std::string &model_container_name) const {
  std::lock_guard<std::mutex> lock{mutex_};
  std::string frame_id;
  if (!model_containers_.count(model_container_name)) {
    throw std::invalid_argument{"There's no container filed with the name " +
                                model_container_name};
  }
  return model_containers_.at(model_container_name)
      .resource()
      ->containerReferenceFrameId();
}

std::string ResourceManager::getContainerExclusionZoneId(
    const std::string &model_container_name) const {
  std::lock_guard<std::mutex> lock{mutex_};
  std::string frame_id;
  if (!model_containers_.count(model_container_name)) {
    throw std::invalid_argument{"There's no container filed with the name " +
                                model_container_name};
  }
  return model_containers_.at(model_container_name)
      .resource()
      ->exclusionZoneId();
}

WorkRegionId
ResourceManager::getWorkRegionId(const ManagedLocusHandle &handle) const {
  auto parent_container_ptr = findLociContainer(*handle.resource());
  if (!parent_container_ptr) {
    throw std::runtime_error("Can't find parent for requested locus");
  }
  return parent_container_ptr->resource()->region();
}

std::optional<ResourceManagerInterface::ManagedLocusHandle>
ResourceManager::getManagedLocusHandleForPose(const RelativePose3 &pose) {
  std::lock_guard<std::mutex> lock{mutex_};
  clearNonAllocatedEmptyLoci();

  // check if the pose matches that of a known managed locus
  auto shortest_distance_to_pose =
      [this,
       &reference_pose = pose](const ManagedLocusHandle &lhs_locus_handle,
                               const ManagedLocusHandle &rhs_locus_handle) {
        auto &lhs_locus = *lhs_locus_handle.resource();
        auto &rhs_locus = *rhs_locus_handle.resource();

        // get all poses in the same frame
        auto lhs_pose = transformPoseToContainerLocalPose(
            lhs_locus.pose(), reference_pose.frameId());
        auto rhs_pose = transformPoseToContainerLocalPose(
            rhs_locus.pose(), reference_pose.frameId());

        // ignore differences in height
        lhs_pose.position().vector().z() =
            reference_pose.position().vector().z();
        rhs_pose.position().vector().z() =
            reference_pose.position().vector().z();

        const auto lhs_distance =
            (lhs_pose.position().vector() - reference_pose.position().vector())
                .norm();
        const auto rhs_distance =
            (rhs_pose.position().vector() - reference_pose.position().vector())
                .norm();

        return lhs_distance < rhs_distance;
      };

  if (model_loci_.size()) {
    // determine the model that's closest to the requested pose
    const auto closest_part = std::min_element(
        model_loci_.begin(), model_loci_.end(), shortest_distance_to_pose);

    // get both poses in the same reference frame
    auto reframed_closest_part_pose = transformPoseToContainerLocalPose(
        closest_part->resource()->pose(), pose.frameId());

    // ignore differences in height
    reframed_closest_part_pose.position().vector().z() =
        pose.position().vector().z();

    // they get compared without rotation, because we may not know the
    // rotation value
    if (Position::samePosition(reframed_closest_part_pose.position(),
                               pose.position(), position_tolerance_)) {
      return *closest_part;
    }
  }

  // none of the existing parts could be matched to the requeste pose.
  // Create a new locus for an empty space.

  auto temporary_locus_handle = ManagedLocus::CreateEmptySpace("", pose);

  auto parent_container = findLociContainer(temporary_locus_handle);

  if (!parent_container) {
    // it does not belong to any container!
    ERROR("Requested a managed locus in a location that does not belong to any "
          "container!");
    return std::nullopt;
  }

  if (!parent_container->resource()->enabled()) {
    // the container is disabled, can't provide a locus from here
    ERROR("Requested a managed locus from a disabled container!");
    return std::nullopt;
  }

  auto parent_name = parent_container->resource()->name();
  auto parent_frame_id =
      parent_container->resource()->containerReferenceFrameId();

  // check if we have surface real state to allocate this
  auto surface_manager = buildContainerSurfaceManager(parent_name);

  auto [x, y, r] = poseToOccupancy(pose, parent_frame_id);

  if (!surface_manager.regionIsWithinSurface(x, y, r) ||
      !surface_manager.regionIsAvailable(x, y, r)) {
    ERROR("Could not create an empty space, there's no free space in {}  "
          "at the requested location",
          parent_container->resource()->name());
    return std::nullopt;
  }

  // we're ok, create a handle for this and return it.
  auto target_locus_handle = ResourceManagerInterface::ManagedLocusHandle(
      std::make_shared<ManagedLocus>(
          ManagedLocus::CreateEmptySpace(parent_name, pose)));

  model_loci_.emplace_back(target_locus_handle);

  return std::optional<ManagedLocusHandle>{std::move(target_locus_handle)};
}

SurfaceManager ResourceManager::volumeToSurface(const CuboidVolume &v) const {
  const auto lrb = v.lowerRightBackCorner();
  const auto ulf = v.upperLeftFrontCorner();
  auto x0 = lrb.vector().x();
  auto y0 = lrb.vector().y();
  auto width = ulf.vector().x() - lrb.vector().x();
  auto height = ulf.vector().y() - lrb.vector().y();
  return SurfaceManager(x0, y0, width, height);
}

SurfaceManager ResourceManager::buildContainerSurfaceManager(
    const std::string &parent_name) const {

  auto handle_ptr = model_containers_.at(parent_name).resource();
  auto parent_pose = handle_ptr->pose();

  auto surface_manager = volumeToSurface(handle_ptr->containerVolume());

  for (const auto &model_locus : model_loci_) {
    auto [x, y, r] = poseToOccupancy(model_locus.resource()->pose(),
                                     handle_ptr->containerReferenceFrameId());
    surface_manager.markAsOccupied(x, y, r);
  }

  return surface_manager;
}

void ResourceManager::updateSensorData(
    const std::vector<ObservedModel> &observed_models) {
  std::lock_guard<std::mutex> lock{mutex_};

  ++sensor_update_count_;

  if (!observed_models.size()) {
    WARNING("No models visible on any sensor! Blackout?");
    return;
  }

  std::vector<ManagedLocus> new_model_loci;

  // create a list of model locus that can be matched to a container
  for (const auto &model : observed_models) {
    auto model_locus = ManagedLocus::CreateOccupiedSpace(
        "", model.pose, model.type, model.broken);

    auto parent_container = findLociContainer(model_locus);

    if (!parent_container) {
      WARNING("A model at {} appears in sensor input but could not be matched "
              "to a model tray",
              model.pose);
      continue;
    }

    // TODO (glpuga) this may be more detrimental than not, now that robots take
    // turns for the airspace above containers
    if (parent_container->allocated()) {
      // we ignore model information in allocated containers, because a robot
      // may be moving stuff around
      continue;
    }

    // this transformation makes all objects relative to the surface. This is
    // the same as the relative to the container, except for the conveyor belt.
    const auto surface_frame_id =
        parent_container->resource()->surfaceReferenceFrameId();
    const auto local_frame_pose =
        transformPoseToContainerLocalPose(model.pose, surface_frame_id);

    auto new_model_locus = ManagedLocus::CreateOccupiedSpace(
        parent_container->resource()->name(),
        RelativePose3{surface_frame_id, local_frame_pose}, model.type,
        model.broken);

    // TODO(glpuga) Conveyor belt hack. I artificially increase the difficulty
    // of parts on conveyor belts to reduce their chances to be picked if there
    // are static parts.
    const auto parent_frame_id =
        parent_container->resource()->containerReferenceFrameId();
    if (parent_frame_id != surface_frame_id) {
      new_model_locus.correctDifficulty(conveyor_belt_start_difficulty_);
    }

    // notice that I discard the previously created ManagedLocus instance
    // because it had no parent name.
    auto it = new_model_loci.emplace_back(std::move(new_model_locus));
  }

  std::set<int32_t> loci_to_keep;

  //
  // move over the known loci in allocated containers, since we'll have
  // no information about those from the camera.
  for (auto &known_model_locus_handle : model_loci_) {
    // allocated objects need to be carried over, both empty and non-empty ones
    if (known_model_locus_handle.allocated()) {
      loci_to_keep.insert(known_model_locus_handle.resource()->uniqueId());
      continue;
    }

    // models in allocated containers are ignored in the input, so
    // we carry them over assuming they are still there.
    auto &known_model_locus = *known_model_locus_handle.resource();
    auto parent_container = findLociContainer(known_model_locus);
    if (parent_container->allocated()) {
      loci_to_keep.insert(known_model_locus_handle.resource()->uniqueId());
      continue;
    }
  }

  //
  // find matches between observed camera models and known model loci. Add new
  // ones, and improve information on the others.
  for (const auto &new_model_locus : new_model_loci) {
    bool found{false};

    for (auto &known_model_locus_handle : model_loci_) {
      auto &known_model_locus = *known_model_locus_handle.resource();

      // make sure both poses are known in the same frame
      auto known_model_locus_pose = known_model_locus.pose();
      auto new_model_locus_pose = transformPoseToContainerLocalPose(
          new_model_locus.pose(), known_model_locus_pose.frameId());

      // ignore differences in height
      new_model_locus_pose.position().vector().z() =
          known_model_locus_pose.position().vector().z();

      // they get compared without rotation, because we may not know the
      // rotation value
      if (Position::samePosition(new_model_locus_pose.position(),
                                 known_model_locus_pose.position(),
                                 position_tolerance_)) {
        // don't update parts that are involved in active tasks
        if (known_model_locus_handle.allocated()) {
          found = true;
          break;
        }

        // both are one an the same (based on position), so we use the new
        // information to improve what we know already about the locus

        if (known_model_locus.isEmpty()) {
          // if we knew nothing, then any new information is valid
          known_model_locus = new_model_locus;
        } else {
          // Combine the new and the old information
          auto [known_part_id, known_broken] = known_model_locus.model();
          auto [new_part_id, new_broken] = new_model_locus.model();

          if (new_part_id != PartId::UnkownPartId) {
            known_part_id = new_part_id;
          }

          // TODO(glpuga) Once broken, a model cannot be considered non-broken
          // again this should be fine for this competition, but not in
          // general. Now knowing if something is broken or not should be an
          // option.
          known_broken = known_broken || new_broken;

          const auto difficulty_value = known_model_locus.difficulty();

          // we always update the pose information with the new information.
          // We know that position is the same, but rotation might have
          // changed.
          auto known_pose = new_model_locus.pose();

          known_model_locus = ManagedLocus::CreateOccupiedSpace(
              new_model_locus.parentName(), known_pose, known_part_id,
              known_broken);

          // TODO(glpuga) this is hackish, fix
          known_model_locus.correctDifficulty(difficulty_value);

          loci_to_keep.emplace(known_model_locus.uniqueId());
        }

        // one way or the other, we already had information on this locus that
        // we merged with new data.
        found = true;
        break;
      }
    }

    if (!found) {
      auto it = model_loci_.emplace_back(
          std::make_shared<ManagedLocus>(new_model_locus));
      loci_to_keep.emplace(it.resource()->uniqueId());
    }
  }

  auto filter = [&loci_to_keep](const ManagedLocusHandle &locus) {
    // remove it if it's not currently in use and it represents
    // an empty locus
    return loci_to_keep.count(locus.resource()->uniqueId()) == 0;
  };

  // erase the loci that have not been updated or that have no special
  // reason to be kept (empty loci)
  model_loci_.erase(
      std::remove_if(model_loci_.begin(), model_loci_.end(), filter),
      model_loci_.end());

  //
  // model_loci_ has now:
  // - Known models located in allocated containers.
  // - New models in non-allocated containers.
  // - known models in non-allocated containers visible in camera.
  // this leaves behind
  // - known models not visible in camera in non-allocated containers
  // - new models in allocated containers
}

const ResourceManager::ModelContainerHandle *
ResourceManager::findLociContainer(const ManagedLocus &locus) const {
  for (const auto &container : model_containers_) {
    if (locusWithinVolume(locus, container.second)) {
      return &container.second;
    }
  }
  return nullptr;
}

void ResourceManager::clearEmptyLoci() {
  auto filter = [](const ManagedLocusHandle &locus) {
    // remove it if it's not currently in use and it represents
    // an empty locus
    return !locus.allocated() && locus.resource()->isEmpty();
  };
  model_loci_.erase(
      std::remove_if(model_loci_.begin(), model_loci_.end(), filter),
      model_loci_.end());
}

Pose3 ResourceManager::transformPoseToContainerLocalPose(
    const RelativePose3 &pose, const std::string &container_frame_id) const {
  auto frame_transformer = toolbox_->getFrameTransformer();
  auto transformed_pose =
      frame_transformer->transformPoseToFrame(pose, container_frame_id);
  return transformed_pose.pose();
}

std::tuple<double, double, double>
ResourceManager::poseToOccupancy(const RelativePose3 &relative_pose,
                                 const std::string &container_frame_id) const {
  Pose3 local_pose =
      transformPoseToContainerLocalPose(relative_pose, container_frame_id);
  auto local_vector = local_pose.position().vector();
  return std::make_tuple(local_vector.x(), local_vector.y(),
                         default_occupancy_radius_);
}

bool ResourceManager::locusWithinVolume(
    const ManagedLocus &locus, const ModelContainerHandle &container) const {
  const auto &container_volume = container.resource()->containerVolume();
  const auto &local_frame_id =
      container.resource()->containerReferenceFrameId();
  const auto local_pose =
      transformPoseToContainerLocalPose(locus.pose(), local_frame_id);
  return container_volume.contains(local_pose.position());
}

void ResourceManager::clearNonAllocatedEmptyLoci() {
  auto filter = [](const ManagedLocusHandle &locus) {
    return !locus.allocated() && locus.resource()->isEmpty();
  };

  model_loci_.erase(
      std::remove_if(model_loci_.begin(), model_loci_.end(), filter),
      model_loci_.end());
}

void ResourceManager::logKnownLoci() {
  std::lock_guard<std::mutex> lock{mutex_};
  clearNonAllocatedEmptyLoci();

  WARNING("Known world state ({} loci)", model_loci_.size());
  for (auto &known_model_locus_handle : model_loci_) {
    auto &known_model_locus = *known_model_locus_handle.resource();

    // Convert to the frame of the parent
    auto known_model_locus_pose = known_model_locus.pose();
    auto parent_frame = model_containers_.at(known_model_locus.parentName())
                            .resource()
                            ->containerReferenceFrameId();
    auto pose_in_parent =
        RelativePose3{parent_frame, transformPoseToContainerLocalPose(
                                        known_model_locus_pose, parent_frame)};
    if (known_model_locus.isEmpty()) {
      INFO(" - empty at {} (allocated: {}, diff: {}, uid: {})", pose_in_parent,
           known_model_locus_handle.allocated(), known_model_locus.difficulty(),
           known_model_locus.uniqueId());
    } else {
      auto [known_model_part_id, broken] = known_model_locus.model();
      (void)broken;
      INFO(" - {} at {} (broken: {} , allocated: {}, diff: {}, uid: {})",
           known_model_part_id.codedString(), pose_in_parent, broken,
           known_model_locus_handle.allocated(), known_model_locus.difficulty(),
           known_model_locus.uniqueId());
    }
  }
  INFO("---");
}

const std::vector<ModelTraySharedAccessSpaceDescription> &
ResourceManager::getListOfExclusionZones() const {
  // no need to take the mutex, since this data is read-only.
  return shared_workspace_descriptors_;
}

} // namespace tijcore
