/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library
#include <string>
#include <utility>

// tijcore
#include <tijcore/utils/StaticFrameTransformer.hpp>

namespace tijcore
{
StaticFrameTransformer::StaticFrameTransformer(std::initializer_list<TransformTreeLink> links)
{
  for (const auto& entry : links)
  {
    transform_to_parent_.emplace(std::make_pair(entry.child_frame_id, entry.PoseInParent));
  }

  // validate the transforms and pre-fill the transform cache
  try
  {
    for (const auto& [child_frame_id, pose] : transform_to_parent_)
    {
      findTransformToRoot(child_frame_id);
      (void)pose;  // avoids unused variable warning
    }
  }
  catch (const std::invalid_argument&)
  {
    throw;
  }
}

tijmath::RelativePose3 StaticFrameTransformer::transformPoseToFrame(const tijmath::RelativePose3& relative_pose,
                                                                    const std::string& new_frame_id) const
{
  auto transform_a_in_w = findTransformToRoot(relative_pose.frameId());
  auto transform_p_in_a = poseToIsometry(relative_pose.pose());
  auto transform_b_in_w = findTransformToRoot(new_frame_id);

  auto transform_p_in_b = transform_b_in_w.inv() * transform_a_in_w * transform_p_in_a;

  return TransformToRelativePose(transform_p_in_b, new_frame_id);
}

tijmath::Isometry StaticFrameTransformer::findTransformToRoot(const std::string& frame_id, const int32_t nesting) const
{
  if (nesting > max_nesting_)
  {
    throw std::invalid_argument{
      "Maximum nesting value reached, there may be a "
      "loop inthe static transform tree!"
    };
  }

  if (frame_id == root_frame_)
  {
    return tijmath::Isometry::Identity;
  }

  if (transform_to_root_cache_.count(frame_id))
  {
    return transform_to_root_cache_.at(frame_id);
  }

  if (!transform_to_parent_.count(frame_id))
  {
    if (nesting == 0)
    {
      // we were on the top level call
      throw std::invalid_argument{ frame_id + " is not a frame id in the static transform tree" };
    }
    else
    {
      // we were on a recursive call to the function
      throw std::invalid_argument{
        "While moving upwards on the static transform tree found a frame id "
        "with no parent link (" +
        frame_id + ") the tree is a disconnected graph"
      };
    }
  }

  auto& pose_in_parent = transform_to_parent_.at(frame_id);
  tijmath::Isometry transform_to_root{ poseToIsometry(pose_in_parent.pose()) };

  if (pose_in_parent.frameId() != root_frame_)
  {
    transform_to_root = findTransformToRoot(pose_in_parent.frameId(), nesting + 1) * transform_to_root;
  }

  transform_to_root_cache_.emplace(std::make_pair(frame_id, transform_to_root));
  return transform_to_root;
}

tijmath::Isometry StaticFrameTransformer::poseToIsometry(const tijmath::Pose3& pose) const
{
  return tijmath::Isometry{ pose.rotation().rotationMatrix(), pose.position().vector() };
}

tijmath::RelativePose3 StaticFrameTransformer::TransformToRelativePose(const tijmath::Isometry& tr,
                                                                       const std::string& parent_frame_id) const
{
  return tijmath::RelativePose3{ parent_frame_id, tijmath::Position{ tr.translation() },
                                 tijmath::Rotation{ tr.rotation() } };
}

}  // namespace tijcore
