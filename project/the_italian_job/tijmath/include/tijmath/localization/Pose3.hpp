/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

#include <tijmath/math/Position.hpp>
#include <tijmath/math/Rotation.hpp>

namespace tijmath
{
class Pose3
{
public:
  static const Pose3 Origin;

  Pose3();

  Pose3(const Position& p, const Rotation& r);

  Position position() const;
  Rotation rotation() const;

  Position& position();
  Rotation& rotation();

  static bool samePose3(const Pose3& lhs, const Pose3& rhs, const double position_tolerance,
                        const double angular_tolerance);

private:
  Position p_{ Vector3{ 0, 0, 0 } };
  Rotation r_{ Quaternion{ 0, 0, 0, 1 } };
};

std::ostream& operator<<(std::ostream& os, const Pose3& v);

}  // namespace tijmath
