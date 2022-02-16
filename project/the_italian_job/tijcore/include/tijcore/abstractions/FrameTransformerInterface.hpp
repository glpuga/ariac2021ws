/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>
#include <string>

// tijcore
#include <tijmath/RelativePose3.hpp>

namespace tijcore
{
class FrameTransformerInterface
{
public:
  using SharedPtr = std::shared_ptr<FrameTransformerInterface>;

  virtual ~FrameTransformerInterface() = default;

  virtual tijmath::RelativePose3 transformPoseToFrame(const tijmath::RelativePose3& pose,
                                                      const std::string& new_frame_id) const = 0;
};

}  // namespace tijcore
