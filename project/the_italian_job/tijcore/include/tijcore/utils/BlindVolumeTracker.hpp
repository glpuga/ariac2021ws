/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// tijcore
#include <tijcore/abstractions/BlindVolumeTrackerInterface.hpp>
#include <tijmath/RelativePose3.hpp>

namespace tijcore
{
class BlindVolumeTracker : public BlindVolumeTrackerInterface
{
public:
  BlindVolumeTracker(const tijmath::RelativePose3& pose, const double radius)
    : pose_{ pose }, radius_{ radius }
  {
  }

  tijmath::RelativePose3 pose() const override
  {
    return pose_;
  }

  double radius() const override
  {
    return radius_;
  }

private:
  tijmath::RelativePose3 pose_;
  double radius_;
};

}  // namespace tijcore
