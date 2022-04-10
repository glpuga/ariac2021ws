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

  std::vector<ManagedLocusHandle> findSourceLociByPartId(const PartId& part_id) override;

  std::vector<ManagedLocusHandle>
  findSiblingLociByCommonParent(const std::string& parent_name) override;

  std::optional<PickAndPlaceRobotHandle> getPickAndPlaceRobotHandle() override;

  std::optional<ManagedLocusHandle>
  createVacantLociAtPose(const tijmath::RelativePose3& pose) override;

  void processInputSensorData(const std::vector<ObservedItem>& observed_models) override;

  void logCurrentResourceManagerState() override;

private:
  using ModelContainerHandle = ResourceHandle<ModelContainerInterface>;

  int32_t sensor_update_count_{ 0 };

  const double position_tolerance_{ 0.024 };  // TODO(glpuga) analyze what value should be used for
                                              // this
  const double default_occupancy_radius_{ 0.02 };  // TODO(glpuga) this fixed value should be
                                                   // replaced by the footprint of a given part, or
                                                   // at least be a function of part size.

  mutable std::mutex mutex_;

  Toolbox::SharedPtr toolbox_;

  std::vector<ManagedLocusHandle> model_loci_;

  std::map<std::string, ModelContainerHandle> model_containers_;

  std::vector<PickAndPlaceRobotHandle> pick_and_place_robots_;

  const ModelContainerHandle* findLociContainer(const ManagedLocus& locus) const;

  void clearEmptyLoci();

  tijmath::Pose3 transformPoseToContainerLocalPose(const tijmath::RelativePose3& relative_pose,
                                                   const std::string& container_frame_id) const;

  std::tuple<double, double, double> poseToOccupancy(const tijmath::RelativePose3& relative_pose,
                                                     const std::string& container_frame_id) const;

  SurfaceManager volumeToSurface(const CuboidVolume& v) const;

  SurfaceManager buildContainerSurfaceManager(const std::string& parent_name) const;

  bool locusWithinVolume(const ManagedLocus& locus, const ModelContainerHandle& container) const;

  void clearNonAllocatedEmptyLoci();
};

}  // namespace tijcore
