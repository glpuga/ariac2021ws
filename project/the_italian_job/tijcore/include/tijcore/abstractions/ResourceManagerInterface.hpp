/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

// tijcore
#include <tijcore/abstractions/PickAndPlaceRobotInterface.hpp>
#include <tijcore/coremodels/SubmissionTrayAdapter.hpp>
#include <tijcore/datatypes/ObservedItem.hpp>
#include <tijcore/resources/ManagedLocus.hpp>
#include <tijcore/resources/ResourceHandle.hpp>

namespace tijcore
{
class ResourceManagerInterface
{
public:
  using Ptr = std::unique_ptr<ResourceManagerInterface>;
  using SharedPtr = std::shared_ptr<ResourceManagerInterface>;

  using ManagedLocusHandle = ResourceHandle<ManagedLocus>;
  using PickAndPlaceRobotHandle = ResourceHandle<PickAndPlaceRobotInterface>;
  using SubmissionTrayHandle = ResourceHandle<SubmissionTrayAdapter>;

  virtual ~ResourceManagerInterface() = default;

  virtual std::vector<ManagedLocusHandle> findVacantLociCandidates(const double free_radius) = 0;

  virtual std::vector<ManagedLocusHandle> findSourceLociByPartId(const PartId& part_id) = 0;

  virtual std::vector<ManagedLocusHandle>
  findSiblingLociByCommonParent(const std::string& parent_name) = 0;

  virtual std::optional<PickAndPlaceRobotHandle> getPickAndPlaceRobotHandle() = 0;

  virtual std::optional<ManagedLocusHandle>
  createVacantLociAtPose(const tijmath::RelativePose3& pose) = 0;

  virtual void processInputSensorData(const std::vector<ObservedItem>& observed_models) = 0;

  virtual void logCurrentResourceManagerState()
  {
  }  // default non-implementation
};

}  // namespace tijcore
