/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>
#include <optional>
#include <string>

// tijcore
#include <tijcore/resources/ResourceManager.hpp>

namespace tijcore
{
struct BTTaskData
{
  using SharedPtr = std::shared_ptr<BTTaskData>;

  std::optional<ResourceManagerInterface::ManagedLocusHandle> src_locus;
  std::optional<ResourceManagerInterface::ManagedLocusHandle> dst_locus;
  std::optional<ResourceManagerInterface::PickAndPlaceRobotHandle> primary_robot;
};

}  // namespace tijcore
