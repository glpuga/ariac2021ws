/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <initializer_list>
#include <map>
#include <string>
#include <vector>

// tijcore
#include <tijcore/math/Isometry.hpp>
#include <tijcore/perception/FrameTransformerInterface.hpp>

namespace tijcore {

class StaticFrameTransformer : public FrameTransformerInterface {
public:
  struct TransformTreeLink {
    std::string child_frame_id;
    RelativePose3 PoseInParent;
  };

  StaticFrameTransformer(const std::initializer_list<TransformTreeLink> links);

  RelativePose3
  transformPoseToFrame(const RelativePose3 &pose,
                       const std::string &new_frame_id) const override;

private:
  const std::string root_frame_{"world"};
  const int32_t max_nesting_{10};

  std::map<std::string, RelativePose3> transform_to_parent_;
  mutable std::map<std::string, Isometry> transform_to_root_cache_;

  Isometry findTransformToRoot(const std::string &,
                               const int32_t nesting = 0) const;

  Isometry poseToIsometry(const Pose3 &pose) const;

  RelativePose3
  TransformToRelativePose(const Isometry &tr,
                          const std::string &parent_frame_id) const;
};

} // namespace tijcore
