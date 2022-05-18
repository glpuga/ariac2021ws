/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>
#include <optional>
#include <string>

// tijcore
#include <tijcore/coremodels/Toolbox.hpp>
#include <tijcore/resources/ResourceManager.hpp>

namespace tijcore
{
struct BTTaskParameters
{
  using SharedPtr = std::shared_ptr<BTTaskParameters>;

  //
  // Task resources

  ResourceManagerInterface::SharedPtr resource_manager;
  Toolbox::SharedPtr toolbox;

  //
  // Task parameters

  std::optional<ResourceManagerInterface::ManagedLocusHandle> src_locus;
  std::optional<ResourceManagerInterface::ManagedLocusHandle> dst_locus;
  std::optional<ResourceManagerInterface::PickAndPlaceRobotHandle> primary_robot;
};

}  // namespace tijcore
