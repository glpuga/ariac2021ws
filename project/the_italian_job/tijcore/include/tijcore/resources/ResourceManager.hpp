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
#include <tijcore/resources/SurfaceManager.hpp>

namespace tijcore
{
class ResourceManager : public ResourceManagerInterface
{
public:
  ResourceManager(const Toolbox::SharedPtr& toolbox,
                  std::vector<ModelContainerInterface::Ptr>&& model_containers,
                  std::vector<PickAndPlaceRobotInterface::Ptr>&& pick_and_place_robots);

  std::vector<ManagedLocusHandle> findVacantLociCandidates(const double free_radius) override;

  std::vector<ManagedLocusHandle> getPartSourceListByType(const PartId& part_id) override;

  std::vector<ManagedLocusHandle>
  getMovableTraySourceListByType(const MovableTrayId& movable_tray_type) override;

  std::vector<ManagedLocusHandle>
  findSiblingLociByCommonParent(const std::string& parent_name) override;

  std::optional<PickAndPlaceRobotHandle> getPickAndPlaceRobotHandle() override;

  std::optional<ManagedLocusHandle> getLocusAtPose(const tijmath::RelativePose3& pose) override;

  void processInputSensorData(const std::vector<ObservedItem>& observed_models) override;

  void logCurrentResourceManagerState() override;

private:
  using ModelContainerHandle = ResourceHandle<ModelContainerInterface>;

  const double locus_identity_position_tolerance_{ 0.024 };  // TODO(glpuga) analyze what value
                                                             // should be used for this
  const double default_occupancy_radius_{ 0.02 };  // TODO(glpuga) this fixed value should be
                                                   // replaced by the footprint of a given part, or
                                                   // at least be a function of part size.

  mutable std::mutex mutex_;

  Toolbox::SharedPtr toolbox_;

  std::vector<ManagedLocusHandle> known_loci_resource_state_;

  std::map<std::string, ModelContainerHandle> model_containers_;

  std::vector<PickAndPlaceRobotHandle> pick_and_place_robots_;

  void clearEmptyLoci();

  tijmath::Pose3 transformPoseToContainerLocalPose(const tijmath::RelativePose3& relative_pose,
                                                   const std::string& container_frame_id) const;

  std::tuple<double, double, double> poseToOccupancy(const tijmath::RelativePose3& relative_pose,
                                                     const std::string& container_frame_id) const;

  SurfaceManager volumeToSurface(const CuboidVolume& v) const;

  SurfaceManager buildContainerSurfaceManager(const std::string& parent_name) const;

  const ModelContainerHandle* getPoseParentContainerPtr(const tijmath::RelativePose3& pose) const;

  bool poseIsOnPartContainer(const tijmath::RelativePose3& pose,
                             const ModelContainerHandle& container) const;

  std::vector<ManagedLocus>
  createObservedLociListFromObservedModels(const std::vector<ObservedItem>& observed_models) const;

  void clearNonAllocatedEmptyLoci();

  ManagedLocus mergeOldAndNewPartLocusInformation(const ManagedLocus& known_locus_data,
                                                  const ManagedLocus& new_locus_data) const;

  ManagedLocus mergeOldAndNewMovableTrayLocusInformation(const ManagedLocus& known_locus_data,
                                                         const ManagedLocus& new_locus_data) const;
};

}  // namespace tijcore
