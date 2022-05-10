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
#include <tijcore/abstractions/PickAndPlaceRobotMovementsInterface.hpp>
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
  using PickAndPlaceRobotHandle = ResourceHandle<PickAndPlaceRobotMovementsInterface>;

  virtual ~ResourceManagerInterface() = default;

  virtual std::vector<ManagedLocusHandle> findVacantLociCandidates(const double free_radius) = 0;

  virtual std::vector<ManagedLocusHandle> getPartSourceListByType(const PartId& part_id) = 0;

  // TODO(glpuga): implement tests for this method. https://github.com/glpuga/ariac2021ws/issues/162
  virtual std::vector<ManagedLocusHandle>
  getMovableTraySourceListByType(const MovableTrayId& movable_tray_type) = 0;

  // TODO(glpuga): remove this method, is not really being needed.
  // https://github.com/glpuga/ariac2021ws/issues/161
  virtual std::vector<ManagedLocusHandle>
  findSiblingLociByCommonParent(const std::string& parent_name) = 0;

  virtual std::optional<PickAndPlaceRobotHandle>
  getPickAndPlaceRobotHandle(const std::vector<tijmath::RelativePose3>& waypoints) = 0;

  virtual std::optional<ManagedLocusHandle> getLocusAtPose(const tijmath::RelativePose3& pose) = 0;

  virtual void processInputSensorData(const std::vector<ObservedItem>& observed_models) = 0;

  virtual void logCurrentResourceManagerState()
  {
  }  // default non-implementation
};

}  // namespace tijcore
