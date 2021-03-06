/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <array>
#include <cstdint>
#include <ostream>

// tijcore
#include <tijcore/math/Matrix3.hpp>
#include <tijcore/math/Quaternion.hpp>
#include <tijcore/math/Vector3.hpp>

namespace tijcore {

class Isometry {
public:
  static const Isometry Identity;

  Isometry();

  Isometry(const Matrix3 &rotation, const Vector3 &translation);

  Isometry(const Isometry &);
  Isometry(Isometry &&);
  Isometry &operator=(const Isometry &);
  Isometry &operator=(Isometry &&);

  Vector3 &translation();
  const Vector3 &translation() const;

  Matrix3 &rotation();
  const Matrix3 &rotation() const;

  Isometry operator*(const Isometry &rhs) const;
  Isometry &operator*=(const Isometry &rhs);

  Isometry inv() const;

private:
  Matrix3 rotation_;
  Vector3 translation_;
};

std::ostream &operator<<(std::ostream &os, const Isometry &m);

} // namespace tijcore
