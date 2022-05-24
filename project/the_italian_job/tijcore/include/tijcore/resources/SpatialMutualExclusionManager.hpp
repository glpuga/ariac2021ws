/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <mutex>
#include <optional>
#include <string>
#include <vector>

// tijcore
#include <tijcore/abstractions/SpatialMutualExclusionManagerInterface.hpp>

namespace tijcore
{
class SpatialMutualExclusionManager : public SpatialMutualExclusionManagerInterface
{
public:
  SpatialMutualExclusionManager(const std::string& preferred_frame_id,
                                FrameTransformerInterface::SharedPtr frame_transformer_instance);

  std::optional<VolumeHandle> lockSphereVolume(const tijmath::RelativePose3& pose,
                                               const double radius) override;

  std::optional<SpatialMutualExclusionManager::VolumeHandle>
  lockSpheresPathVolume(const tijmath::RelativePose3& start_pose,
                        const tijmath::RelativePose3& end_pose, const double radius) override;

private:
  struct VolumeDataEntry
  {
    tijutils::UniqueId uid;
    std::vector<tijmath::Pose3> centers;
    double radius;
    VolumeHandle handle;
  };

  std::string preferred_frame_id_;
  FrameTransformerInterface::SharedPtr frame_transformer_instance_;

  std::vector<VolumeDataEntry> volume_data_;

  std::mutex mutex_;

  void pruneReleasedVolumes();

  bool volumeIsTakenAlready(const tijmath::Pose3& preferred_frame_pose, const double radius);

  std::optional<SpatialMutualExclusionManager::VolumeHandle>
  lockSphereVolumesPrivate(const std::vector<tijmath::Pose3>& centers, const double radius);
};

}  // namespace tijcore
