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
#include <tijcore/datatypes/ObservedModel.hpp>
#include <tijcore/datatypes/WorkRegionId.hpp>
#include <tijcore/resources/ManagedLocus.hpp>
#include <tijcore/resources/ModelTraySharedAccessSpaceDescription.hpp>
#include <tijcore/resources/ModelTraySharedAccessSpaceResource.hpp>
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
  using ExclusionZoneHandle = ResourceHandle<ModelTraySharedAccessSpaceResource>;
  using SubmissionTrayHandle = ResourceHandle<SubmissionTrayAdapter>;

  virtual ~ResourceManagerInterface() = default;

  virtual std::vector<ManagedLocusHandle> findEmptyLoci(const double free_radius) = 0;

  virtual std::vector<ManagedLocusHandle> findManagedLociByPartId(const PartId& part_id) = 0;

  virtual std::vector<ManagedLocusHandle>
  findManagedLociByParent(const std::string& parent_name) = 0;

  virtual std::optional<PickAndPlaceRobotHandle>
  getPickAndPlaceRobotHandle(const std::set<WorkRegionId>& regions) = 0;

  virtual std::optional<ManagedLocusHandle>
  getManagedLocusHandleForPose(const tijmath::RelativePose3& pose) = 0;

  virtual void updateSensorData(const std::vector<ObservedModel>& observed_models) = 0;

  virtual void logKnownLoci()
  {
  }  // default non-implementation
};

}  // namespace tijcore