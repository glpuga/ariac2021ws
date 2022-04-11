/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>

// gtest
#include "gmock/gmock.h"

// tijcore
#include <tijcore/abstractions/BlindVolumeTrackerInterface.hpp>

namespace tijcore
{
class BlindVolumeTrackerMock : public BlindVolumeTrackerInterface
{
public:
  using Ptr = std::unique_ptr<BlindVolumeTrackerMock>;

  MOCK_CONST_METHOD0(pose, tijmath::RelativePose3());

  MOCK_CONST_METHOD0(radius, double());
};

}  // namespace tijcore
