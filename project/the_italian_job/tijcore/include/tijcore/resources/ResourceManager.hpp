/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <initializer_list>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <vector>

// tijcore
#include <tijcore/abstractions/ModelContainerInterface.hpp>
#include <tijcore/abstractions/ResourceManagerInterface.hpp>
#include <tijcore/coremodels/Toolbox.hpp>
#include <tijcore/resources/ModelTraySharedAccessSpaceDescription.hpp>
#include <tijcore/resources/SurfaceManager.hpp>

namespace tijcore
{
class ResourceManager : public ResourceManagerInterface
{
public:
  ResourceManager(const Toolbox::SharedPtr& toolbox,
                  const std::vector<ModelTraySharedAccessSpaceDescription>& shared_workspaces,
                  std::vector<ModelContainerInterface::Ptr>&& model_containers,
                  std::vector<PickAndPlaceRobotInterface::Ptr>&& pick_and_place_robots);

  std::vector<ManagedLocusHandle> findEmptyLoci(const double free_radius) override;

  std::vector<ManagedLocusHandle> findManagedLociByPartId(const PartId& part_id) override;

  std::vector<ManagedLocusHandle> findManagedLociByParent(const std::string& parent_name) override;

  std::optional<PickAndPlaceRobotHandle>
  getPickAndPlaceRobotHandle(const std::set<WorkRegionId>& regions) override;

  std::optional<ManagedLocusHandle>
  getManagedLocusHandleForPose(const tijmath::RelativePose3& pose) override;

  void updateSensorData(const std::vector<ObservedModel>& observed_models) override;

  std::optional<ExclusionZoneHandle>
  getModelTrayExclusionZoneHandle(const std::string& model_container_name,
                                  const std::optional<std::string>& additional_model_tray_name_opt,
                                  const std::chrono::milliseconds& timeout) override;

  std::optional<SubmissionTrayHandle>
  getSubmissionTray(const std::string& model_container_name) override;

  std::string getContainerExclusionZoneId(const std::string& model_container_name) const override;

  WorkRegionId getWorkRegionId(const ManagedLocusHandle& handle) const override;

  void logKnownLoci() override;

  const std::vector<ModelTraySharedAccessSpaceDescription>&
  getListOfExclusionZones() const override;

private:
  using ModelContainerHandle = ResourceHandle<ModelContainerInterface>;
  using SharedWorkspaceHandle = ResourceHandle<std::string>;

  int32_t sensor_update_count_{ 0 };

  const double position_tolerance_{ 0.024 };  // TODO(glpuga) analyze what value should be used for
                                              // this
  const double default_occupancy_radius_{ 0.02 };  // TODO(glpuga) this fixed value should be
                                                   // replaced by the footprint of a given part, or
                                                   // at least be a function of part size.

  const std::vector<ModelTraySharedAccessSpaceDescription> shared_workspace_descriptors_;

  mutable std::mutex mutex_;

  Toolbox::SharedPtr toolbox_;

  std::vector<ManagedLocusHandle> model_loci_;

  std::map<std::string, ModelContainerHandle> model_containers_;
  // std::map<std::string, ExclusionZoneHandle> working_spaces_;
  std::map<std::string, SharedWorkspaceHandle> shared_working_spaces_;

  std::vector<PickAndPlaceRobotHandle> pick_and_place_robots_;

  const ModelContainerHandle* findLociContainer(const ManagedLocus& locus) const;

  void clearEmptyLoci();

  tijmath::Pose3 transformPoseToContainerLocalPose(const tijmath::RelativePose3& relative_pose,
                                                   const std::string& container_frame_id) const;

  std::tuple<double, double, double> poseToOccupancy(const tijmath::RelativePose3& relative_pose,
                                                     const std::string& container_frame_id) const;

  SurfaceManager buildContainerSurfaceManager(const std::string& parent_name) const;

  SurfaceManager volumeToSurface(const CuboidVolume& v) const;

  bool locusWithinVolume(const ManagedLocus& locus, const ModelContainerHandle& container) const;

  void clearNonAllocatedEmptyLoci();
};

}  // namespace tijcore
