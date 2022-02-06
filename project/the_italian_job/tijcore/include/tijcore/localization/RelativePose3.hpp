/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

#include <tijcore/localization/FrameId.hpp>
#include <tijcore/localization/Pose3.hpp>

namespace tijcore
{
class RelativePose3
{
public:
  static const RelativePose3 Origin;

  RelativePose3();

  explicit RelativePose3(const FrameId& frame_id);

  RelativePose3(const FrameId& frame_id, const Pose3& pose);

  RelativePose3(const FrameId& frame_id, const Position& p, const Rotation& r) : frame_id_{ frame_id }, pose_{ p, r }
  {
  }

  const FrameId& frameId() const
  {
    return frame_id_;
  }
  FrameId& frameId()
  {
    return frame_id_;
  }

  Pose3 pose() const
  {
    return pose_;
  }
  Pose3& pose()
  {
    return pose_;
  }

  Position position() const
  {
    return pose_.position();
  }
  Rotation rotation() const
  {
    return pose_.rotation();
  }

  Position& position()
  {
    return pose_.position();
  }
  Rotation& rotation()
  {
    return pose_.rotation();
  }

  static bool sameRelativePose3(const RelativePose3& lhs, const RelativePose3& rhs, const double position_tolerance,
                                const double angular_tolerance);

private:
  FrameId frame_id_;
  Pose3 pose_;
};

std::ostream& operator<<(std::ostream& os, const RelativePose3& v);

}  // namespace tijcore
