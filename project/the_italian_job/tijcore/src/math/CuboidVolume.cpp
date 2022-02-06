/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// tijcore
#include <tijcore/math/CuboidVolume.hpp>

namespace tijcore
{
CuboidVolume::CuboidVolume() : CuboidVolume(Vector3{}, Vector3{})
{
}

CuboidVolume::CuboidVolume(const Vector3& lower_right_back, const Vector3& upper_left_front)
  : lower_right_back_{ lower_right_back }, upper_left_front_{ upper_left_front }
{
}

bool CuboidVolume::contains(const Vector3& point) const
{
  auto a_less_than_x_less_than_b = [](const double a, const double x, const double b) { return (a <= x) && (x <= b); };
  return a_less_than_x_less_than_b(lower_right_back_.x(), point.x(), upper_left_front_.x()) &&
         a_less_than_x_less_than_b(lower_right_back_.y(), point.y(), upper_left_front_.y()) &&
         a_less_than_x_less_than_b(lower_right_back_.z(), point.z(), upper_left_front_.z());
}

bool CuboidVolume::contains(const Position& point) const
{
  return contains(point.vector());
}

bool CuboidVolume::contains(const Pose3& point) const
{
  return contains(point.position().vector());
}

Position CuboidVolume::lowerRightBackCorner() const
{
  return Position{ lower_right_back_ };
}
Position CuboidVolume::upperLeftFrontCorner() const
{
  return Position{ upper_left_front_ };
}

}  // namespace tijcore
