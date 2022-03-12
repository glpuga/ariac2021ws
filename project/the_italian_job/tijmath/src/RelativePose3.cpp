/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library

// tijmath
#include <tijmath/RelativePose3.hpp>

namespace tijmath
{
const RelativePose3 RelativePose3::Origin{};

RelativePose3::RelativePose3() = default;

RelativePose3::RelativePose3(const FrameId& frame_id) : frame_id_{ frame_id }
{
}

RelativePose3::RelativePose3(const FrameId& frame_id, const Pose3& pose)
  : frame_id_{ frame_id }, pose_{ pose }
{
}

bool RelativePose3::sameRelativePose3(const RelativePose3& lhs, const RelativePose3& rhs,
                                      const double position_tolerance,
                                      const double angular_tolerance)
{
  return (lhs.frameId() == rhs.frameId()) &&
         Position::samePosition(lhs.position(), rhs.position(), position_tolerance) &&
         Rotation::sameRotation(lhs.rotation(), rhs.rotation(), angular_tolerance);
}

std::ostream& operator<<(std::ostream& os, const RelativePose3& p)
{
  os << "{f: " << p.frameId() << ", p: " << p.position() << " r: " << p.rotation() << "}";
  return os;
}

}  // namespace tijmath
