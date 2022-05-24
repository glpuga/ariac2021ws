/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <algorithm>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

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

bool SpatialMutualExclusionManager::volumeIsTakenAlready(const tijmath::Pose3& test_center,
                                                         const double radius)
{
  for (const auto& volume_data_entry : volume_data_)
  {
    for (const auto& center : volume_data_entry.centers)
    {
      const auto relative_vector = center.position().vector() - test_center.position().vector();
      const auto distance = relative_vector.norm();

      if (distance <= volume_data_entry.radius + radius)
      {
        return true;
      }
    }
  }

  return false;
}

std::optional<SpatialMutualExclusionManager::VolumeHandle>
SpatialMutualExclusionManager::lockSphereVolume(const tijmath::RelativePose3& pose,
                                                const double radius)
{
  // notice we do not take the mutex here, because we take it in lockSphereVolumesPrivate
  const auto preferred_frame_pose =
      frame_transformer_instance_->transformPoseToFrame(pose, preferred_frame_id_);
  return lockSphereVolumesPrivate({ preferred_frame_pose.pose() }, radius);
}

std::optional<SpatialMutualExclusionManager::VolumeHandle>
SpatialMutualExclusionManager::lockSpheresPathVolume(const tijmath::RelativePose3& start_pose,
                                                     const tijmath::RelativePose3& end_pose,
                                                     const double radius)
{
  // notice we do not take the mutex here, because we take it in lockSphereVolumesPrivate
  const auto preferred_frame_start_pose =
      frame_transformer_instance_->transformPoseToFrame(start_pose, preferred_frame_id_);

  const auto preferred_frame_end_pose =
      frame_transformer_instance_->transformPoseToFrame(end_pose, preferred_frame_id_);

  const auto start_to_end_vector =
      preferred_frame_end_pose.position().vector() - preferred_frame_start_pose.position().vector();

  const auto distance = start_to_end_vector.norm();

  // the 10 factor is just a rule of thumb
  const auto steps = std::max(1, static_cast<int>(distance * 10.0 / radius));

  // 0 to and including steps includes both ends of the path in the vector
  std::vector<tijmath::Pose3> centers;
  for (int i = 1; i <= steps; ++i)
  {
    const auto step_vector = start_to_end_vector * (i / static_cast<double>(steps));
    const auto step_pose = tijmath::Pose3(
        tijmath::Position{ preferred_frame_start_pose.position().vector() + step_vector }, {});
    centers.emplace_back(step_pose);
  }
  return lockSphereVolumesPrivate(centers, radius);
}

std::optional<SpatialMutualExclusionManager::VolumeHandle>
SpatialMutualExclusionManager::lockSphereVolumesPrivate(const std::vector<tijmath::Pose3>& centers,
                                                        const double radius)
{
  std::lock_guard<std::mutex> lock{ mutex_ };

  // remove data from any volumes that have already been released
  pruneReleasedVolumes();

  // check if the volume is already locked
  const bool a_few_of_the_volumes_are_taken =
      std::any_of(centers.begin(), centers.end(), [this, radius](const auto& center) {
        return volumeIsTakenAlready(center, radius);
      });

  if (a_few_of_the_volumes_are_taken)
  {
    return std::nullopt;
  }

  // create a new volume and add it to the vector
  const auto uid = tijutils::UniqueId::CreateNewId();

  VolumeDataEntry new_entry{
    uid,
    centers,                                    //
    radius,                                     //
    VolumeHandle{ std::make_unique<VHData>() }  //
  };

  volume_data_.emplace_back(std::move(new_entry));

  const auto it = volume_data_.end();
  return (it - 1)->handle;
}

}  // namespace tijcore
