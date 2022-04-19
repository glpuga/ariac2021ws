/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

// tijcore
#include <tijcore/resources/SpatialMutualExclusionManager.hpp>
#include <tijlogger/logger.hpp>

namespace tijcore
{
SpatialMutualExclusionManager::SpatialMutualExclusionManager(
    const std::string& preferred_frame_id,
    FrameTransformerInterface::SharedPtr frame_transformer_instance)
  : preferred_frame_id_{ preferred_frame_id }
  , frame_transformer_instance_{ frame_transformer_instance }
{
}

void SpatialMutualExclusionManager::pruneReleasedVolumes()
{
  auto volume_has_been_released = [](const VolumeDataEntry& entry) {
    return !entry.handle.allocated();
  };
  volume_data_.erase(
      std::remove_if(volume_data_.begin(), volume_data_.end(), volume_has_been_released),
      volume_data_.end());
}

bool SpatialMutualExclusionManager::volumeIsTakenAlready(
    const tijmath::RelativePose3& preferred_frame_pose, const double radius)
{
  for (const auto& volume_data_entry : volume_data_)
  {
    const auto relative_vector =
        volume_data_entry.pose.position().vector() - preferred_frame_pose.position().vector();
    const auto distance = relative_vector.norm();

    if (distance <= volume_data_entry.radius + radius)
    {
      return true;
    }
  }

  return false;
}

std::optional<SpatialMutualExclusionManager::VolumeHandle>
SpatialMutualExclusionManager::lockVolume(const tijmath::RelativePose3& pose, const double radius)
{
  std::lock_guard<std::mutex> lock{ mutex_ };

  const auto preferred_frame_pose =
      frame_transformer_instance_->transformPoseToFrame(pose, preferred_frame_id_);

  // remove data from any volumes that have already been released
  pruneReleasedVolumes();

  // check if the volume is already locked
  if (volumeIsTakenAlready(preferred_frame_pose, radius))
  {
    return std::nullopt;
  }

  // create a new volume and add it to the vector
  const auto uid = tijutils::UniqueId::CreateNewId();
  VolumeDataEntry new_entry{
    uid,
    preferred_frame_pose,                       //
    radius,                                     //
    VolumeHandle{ std::make_unique<VHData>() }  //
  };

  volume_data_.emplace_back(std::move(new_entry));

  const auto it = volume_data_.end();
  return (it - 1)->handle;
}

}  // namespace tijcore
