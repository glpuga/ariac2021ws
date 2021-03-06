/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>

// tijcore
#include <tijcore/localization/RelativePose3.hpp>

namespace tijcore {

class FrameTransformerInterface {
public:
  using SharedPtr = std::shared_ptr<FrameTransformerInterface>;

  virtual ~FrameTransformerInterface() = default;

  virtual RelativePose3
  transformPoseToFrame(const RelativePose3 &pose,
                       const std::string &new_frame_id) const = 0;
};

} // namespace tijcore
