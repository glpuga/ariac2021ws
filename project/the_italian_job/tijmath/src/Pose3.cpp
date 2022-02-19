/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library

// tijmath
#include <tijmath/Pose3.hpp>

namespace tijmath
{
const Pose3 Pose3::Origin{};

Pose3::Pose3() = default;

Pose3::Pose3(const Position& p, const Rotation& r) : p_{ p }, r_{ r }
{
}

Position Pose3::position() const
{
  return p_;
}
Rotation Pose3::rotation() const
{
  return r_;
}

Position& Pose3::position()
{
  return p_;
}
Rotation& Pose3::rotation()
{
  return r_;
}

bool samePose3(const Pose3& lhs, const Pose3& rhs, const double position_tolerance,
               const double angular_tolerance)
{
  return Position::samePosition(lhs.position(), rhs.position(), position_tolerance) &&
         Rotation::sameRotation(lhs.rotation(), rhs.rotation(), angular_tolerance);
}

std::ostream& operator<<(std::ostream& os, const Pose3& p)
{
  os << "{ p: " << p.position() << " r: " << p.rotation() << "}";
  return os;
}

}  // namespace tijmath
