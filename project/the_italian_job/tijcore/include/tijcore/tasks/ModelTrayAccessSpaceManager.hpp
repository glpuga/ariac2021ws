/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <chrono>
#include <optional>
#include <string>
#include <vector>

// tijcore
#include <tijcore/perception/PickAndPlaceRobotInterface.hpp>
#include <tijcore/perception/ResourceManagerInterface.hpp>

namespace tijcore {

class ModelTrayAccessSpaceManager {
public:
  ModelTrayAccessSpaceManager(ResourceManagerInterface &resource_manager,
                              PickAndPlaceRobotInterface &robot)
      : resource_manager_{resource_manager}, robot_{robot} {}

  bool getAccessToModel(const std::string &target_parent,
                        std::chrono::milliseconds timeout) {
    return getAccessToModelImpl(target_parent, std::nullopt, timeout);
  }

  bool getAccessToModel(const std::string &first_target_parent,
                        const std::string &second_target_parent,
                        std::chrono::milliseconds timeout) {
    return getAccessToModelImpl(first_target_parent, second_target_parent,
                                timeout);
  }

  void clearAllExclusionZones() {
    robot_.markAsAccessible(resource_manager_.getListOfExclusionZones());
  }

  bool releaseAccess() {
    robot_.markAsInaccessible(resource_manager_.getListOfExclusionZones());
    handle_.reset();
    return true;
  }

private:
  ResourceManagerInterface &resource_manager_;
  PickAndPlaceRobotInterface &robot_;

  std::optional<ResourceManagerInterface::ExclusionZoneHandle> handle_;

  bool getAccessToModelImpl(
      const std::string &first_target_parent,
      const std::optional<std::string> &second_target_parent_opt,
      std::chrono::milliseconds timeout) {
    handle_.reset();

    handle_ = resource_manager_.getModelTrayExclusionZoneHandle(
        first_target_parent, second_target_parent_opt, timeout);
    if (!handle_) {
      return false;
    }

    const auto access_space_id = handle_->resource()->sharedSpaceId();
    auto model_tray_access_space_descriptors =
        resource_manager_.getListOfExclusionZones();

    // split the list between accessible and inaccessible. There has to be a
    // better way to do this in algorithms.h
    std::vector<ModelTraySharedAccessSpaceDescription> inaccessible;
    std::vector<ModelTraySharedAccessSpaceDescription> accessible;
    for (const auto &item : model_tray_access_space_descriptors) {
      if (item.id == access_space_id) {
        accessible.push_back(item);
      } else {
        inaccessible.push_back(item);
      }
    }

    robot_.markAsInaccessible(inaccessible);
    robot_.markAsAccessible(accessible);
    return true;
  }
};

} // namespace tijcore
