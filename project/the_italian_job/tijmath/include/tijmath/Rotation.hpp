/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

#include <tijmath/Matrix3.hpp>
#include <tijmath/Quaternion.hpp>

namespace tijmath
{
class Rotation
{
public:
  static const Rotation Identity;

  Rotation();

  explicit Rotation(const Quaternion& quaternion);

  explicit Rotation(const Matrix3& rotation_matrix);

  Quaternion quaternion() const;

  Matrix3 rotationMatrix() const;

  static Rotation fromQuaternion(const double x, const double y, const double z, const double w);
  static Rotation fromRollPitchYaw(const double roll, const double pitch, const double yaw);

  static bool sameRotation(const Rotation& lhs, const Rotation& rhs, const double tolerance);

private:
  Quaternion quaternion_{ 0, 0, 0, 1 };
};

std::ostream& operator<<(std::ostream& os, const Rotation& r);

}  // namespace tijmath
