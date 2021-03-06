/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>
#include <optional>
#include <set>
#include <vector>

// tijcore
#include <tijcore/agents/ObservedModel.hpp>
#include <tijcore/agents/WorkRegionId.hpp>
#include <tijcore/perception/ManagedLocus.hpp>
#include <tijcore/perception/ModelTraySharedAccessSpaceDescription.hpp>
#include <tijcore/perception/ModelTraySharedAccessSpaceResource.hpp>
#include <tijcore/perception/PickAndPlaceRobotInterface.hpp>
#include <tijcore/perception/ResourceHandle.hpp>
#include <tijcore/perception/SubmissionTrayAdapter.hpp>

namespace tijcore {

class ResourceManagerInterface {
public:
  using Ptr = std::unique_ptr<ResourceManagerInterface>;
  using SharedPtr = std::shared_ptr<ResourceManagerInterface>;

  using ManagedLocusHandle = ResourceHandle<ManagedLocus>;
  using PickAndPlaceRobotHandle = ResourceHandle<PickAndPlaceRobotInterface>;
  using ExclusionZoneHandle =
      ResourceHandle<ModelTraySharedAccessSpaceResource>;
  using SubmissionTrayHandle = ResourceHandle<SubmissionTrayAdapter>;

  virtual ~ResourceManagerInterface() = default;

  virtual std::vector<ManagedLocusHandle> findEmptyLoci() = 0;

  virtual std::vector<ManagedLocusHandle>
  findManagedLociByPartId(const PartId &part_id) = 0;

  virtual std::vector<ManagedLocusHandle>
  findManagedLociByParent(const std::string &parent_name) = 0;

  virtual std::optional<PickAndPlaceRobotHandle>
  getPickAndPlaceRobotHandle(const std::set<WorkRegionId> &regions) = 0;

  virtual std::optional<ManagedLocusHandle>
  getManagedLocusHandleForPose(const RelativePose3 &pose) = 0;

  virtual void
  updateSensorData(const std::vector<ObservedModel> &observed_models) = 0;

  virtual std::optional<ExclusionZoneHandle> getModelTrayExclusionZoneHandle(
      const std::string &model_container_name,
      const std::optional<std::string> &additional_model_tray_name_opt,
      const std::chrono::milliseconds &timeout) = 0;

  virtual std::optional<SubmissionTrayHandle>
  getSubmissionTray(const std::string &model_container_name) = 0;

  virtual std::string
  getContainerFrameId(const std::string &model_container_name) const = 0;

  virtual std::string getContainerExclusionZoneId(
      const std::string &model_container_name) const = 0;

  virtual WorkRegionId
  getWorkRegionId(const ManagedLocusHandle &handle) const = 0;

  virtual const std::vector<ModelTraySharedAccessSpaceDescription> &
  getListOfExclusionZones() const = 0;

  virtual void logKnownLoci() {} // default non-implementation
};

} // namespace tijcore
