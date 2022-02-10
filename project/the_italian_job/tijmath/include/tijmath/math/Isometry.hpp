/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <array>
#include <cstdint>
#include <ostream>

// tijmath
#include <tijmath/math/Matrix3.hpp>
#include <tijmath/math/Quaternion.hpp>
#include <tijmath/math/Vector3.hpp>

namespace tijmath
{
class Isometry
{
public:
  static const Isometry Identity;

  Isometry();

  Isometry(const Matrix3& rotation, const Vector3& translation);

  Isometry(const Isometry&);
  Isometry(Isometry&&);
  Isometry& operator=(const Isometry&);
  Isometry& operator=(Isometry&&);

  Vector3& translation();
  const Vector3& translation() const;

  Matrix3& rotation();
  const Matrix3& rotation() const;

  Isometry operator*(const Isometry& rhs) const;
  Isometry& operator*=(const Isometry& rhs);

  Isometry inv() const;

private:
  Matrix3 rotation_;
  Vector3 translation_;
};

std::ostream& operator<<(std::ostream& os, const Isometry& m);

}  // namespace tijmath
