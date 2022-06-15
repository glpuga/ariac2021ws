
/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>
#include <optional>
#include <string>

// project
#include <tijmath/RelativePose3.hpp>

namespace tijcore
{
class ConveyorBeltManagerInterface
{
public:
  using SharedPtr = std::shared_ptr<ConveyorBeltManagerInterface>;

  struct DetectionData
  {
    double detection_timestamp;
  };

  virtual ~ConveyorBeltManagerInterface() = default;

  virtual std::optional<DetectionData> popNextDetectionTimestamp() = 0;

  virtual tijmath::RelativePose3
  currentPositionInTheBelt(const DetectionData& detection_data) const = 0;

  virtual double getBeltSpeed() const = 0;
};

}  // namespace tijcore
