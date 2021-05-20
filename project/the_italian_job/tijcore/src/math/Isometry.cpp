/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library
#include <cmath>
#include <ostream>
#include <stdexcept>

// tijcore
#include <tijcore/math/Isometry.hpp>

namespace tijcore {

const Isometry Isometry::Identity{
    Matrix3{Vector3{1, 0, 0}, Vector3{0, 1, 0}, Vector3{0, 0, 1}},
    Vector3{0, 0, 0}};

Isometry::Isometry()
    : rotation_{Matrix3::Identity}, translation_{Vector3::Zero} {}

Isometry::Isometry(const Matrix3 &rotation, const Vector3 &translation)
    : rotation_{rotation}, translation_{translation} {}

Isometry::Isometry(const Isometry &) = default;
Isometry::Isometry(Isometry &&) = default;
Isometry &Isometry::operator=(const Isometry &) = default;
Isometry &Isometry::operator=(Isometry &&) = default;

Vector3 &Isometry::translation() { return translation_; }
const Vector3 &Isometry::translation() const { return translation_; }

Matrix3 &Isometry::rotation() { return rotation_; }
const Matrix3 &Isometry::rotation() const { return rotation_; }

Isometry Isometry::operator*(const Isometry &rhs) const {
  Isometry aux{*this};
  aux *= rhs;
  return aux;
}

Isometry Isometry::inv() const {
  auto inv_rotation = rotation_.inv();
  auto inv_translation = (-1) * inv_rotation * translation_;
  return Isometry{inv_rotation, inv_translation};
}

Isometry &Isometry::operator*=(const Isometry &rhs) {
  // order matters
  translation_ += rotation_ * rhs.translation();
  rotation_ *= rhs.rotation();
  return *this;
}

std::ostream &operator<<(std::ostream &os, const Isometry &m) {
  return os << "I{ " << m.rotation() << " " << m.translation() << " }";
}

} // namespace tijcore
