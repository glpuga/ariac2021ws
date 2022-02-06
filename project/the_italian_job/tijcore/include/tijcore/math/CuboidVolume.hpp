/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// tijcore
#include <tijcore/localization/Pose3.hpp>
#include <tijcore/math/Vector3.hpp>

namespace tijcore
{
class CuboidVolume
{
public:
  CuboidVolume();

  CuboidVolume(const Vector3& lower_right_back, const Vector3& upper_left_front);

  bool contains(const Vector3& point) const;
  bool contains(const Position& point) const;
  bool contains(const Pose3& point) const;

  Position lowerRightBackCorner() const;
  Position upperLeftFrontCorner() const;

private:
  Vector3 lower_right_back_;
  Vector3 upper_left_front_;
};

}  // namespace tijcore
