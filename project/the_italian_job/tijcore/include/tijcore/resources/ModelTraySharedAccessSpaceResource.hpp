/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>
#include <string>
#include <vector>

// tijcore
#include <tijcore/abstractions/ModelContainerInterface.hpp>
#include <tijcore/resources/ResourceHandle.hpp>

namespace tijcore
{
class ModelTraySharedAccessSpaceResource
{
public:
  using ModelContainerHandle = ResourceHandle<ModelContainerInterface>;
  using SharedWorkspaceHandle = ResourceHandle<std::string>;

  ModelTraySharedAccessSpaceResource(const std::vector<ModelContainerHandle>& model_tray_handles,
                                     const SharedWorkspaceHandle& shared_access_space_handle)
    : model_tray_handles_{ model_tray_handles }, shared_access_space_handle_{ shared_access_space_handle }
  {
  }

  std::string sharedSpaceId() const
  {
    // if there are multiple handles, they all share the same exlusion zone id
    return model_tray_handles_[0].resource()->exclusionZoneId();
  }

private:
  std::vector<ModelContainerHandle> model_tray_handles_;
  SharedWorkspaceHandle shared_access_space_handle_;
};

}  // namespace tijcore
