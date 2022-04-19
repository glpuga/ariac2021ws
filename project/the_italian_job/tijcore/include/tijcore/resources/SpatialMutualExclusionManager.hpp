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

  std::optional<VolumeHandle> lockVolume(const tijmath::RelativePose3& pose,
                                         const double radius) override;

private:
  struct VolumeDataEntry
  {
    tijutils::UniqueId uid;
    tijmath::RelativePose3 pose;
    double radius;
    VolumeHandle handle;
  };

  std::string preferred_frame_id_;
  FrameTransformerInterface::SharedPtr frame_transformer_instance_;

  std::vector<VolumeDataEntry> volume_data_;

  std::mutex mutex_;

  void pruneReleasedVolumes();

  bool volumeIsTakenAlready(const tijmath::RelativePose3& preferred_frame_pose,
                            const double radius);
};

}  // namespace tijcore
