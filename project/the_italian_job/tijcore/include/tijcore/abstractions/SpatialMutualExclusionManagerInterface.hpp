/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <map>
#include <memory>
#include <optional>
#include <string>

// tijcore
#include <tijcore/abstractions/FrameTransformerInterface.hpp>
#include <tijcore/resources/ResourceHandle.hpp>
#include <tijmath/RelativePose3.hpp>
#include <tijutils/UniqueId.hpp>

namespace tijcore
{
class SpatialMutualExclusionManagerInterface
{
public:
  using Ptr = std::unique_ptr<SpatialMutualExclusionManagerInterface>;
  using SharedPtr = std::shared_ptr<SpatialMutualExclusionManagerInterface>;

  struct VHData
  {
  };
  using VolumeHandle = ResourceHandle<VHData>;

  virtual ~SpatialMutualExclusionManagerInterface() = default;

  virtual std::optional<VolumeHandle> lockSphereVolume(const tijmath::RelativePose3& pose,
                                                       const double radius) = 0;

  virtual std::optional<VolumeHandle>
  lockSpheresPathVolume(const tijmath::RelativePose3& start_pose,
                        const tijmath::RelativePose3& end_pose, const double radius) = 0;
};

}  // namespace tijcore
