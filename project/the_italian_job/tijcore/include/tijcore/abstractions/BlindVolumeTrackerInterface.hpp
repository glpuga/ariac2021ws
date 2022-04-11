/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>

// tijcore
#include <tijmath/RelativePose3.hpp>

namespace tijcore
{
class BlindVolumeTrackerInterface
{
public:
  using Ptr = std::unique_ptr<BlindVolumeTrackerInterface>;

  virtual ~BlindVolumeTrackerInterface() = default;

  virtual tijmath::RelativePose3 pose() const = 0;

  virtual double radius() const = 0;
};

}  // namespace tijcore
