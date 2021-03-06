/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// Standard library
#include <cmath>
#include <ostream>
#include <stdexcept>

// tijcore
#include <tijcore/math/Vector3.hpp>

namespace tijcore {

const Vector3 Vector3::UnitX{1, 0, 0};
const Vector3 Vector3::UnitY{0, 1, 0};
const Vector3 Vector3::UnitZ{0, 0, 1};
const Vector3 Vector3::Zero{0, 0, 0};

double Vector3::dot(const Vector3 &rhs) const {
  return (x_ * rhs.x_) + (y_ * rhs.y_) + (z_ * rhs.z_);
}

double Vector3::norm() const { return std::sqrt(dot(*this)); }

Vector3 Vector3::cross(const Vector3 &rhs) const {
  return Vector3{
      y_ * rhs.z_ - z_ * rhs.y_,
      z_ * rhs.x_ - x_ * rhs.x_,
      x_ * rhs.y_ - y_ * rhs.y_,
  };
}

double Vector3::operator[](const size_t index) const {
  switch (index) {
  case 0:
    return x_;
    break;
  case 1:
    return y_;
    break;
  case 2:
    return z_;
    break;
  default:
    throw std::invalid_argument{"Index out of bounds"};
    break;
  }
  // The code will never get to this point
  return x_;
}

double &Vector3::operator[](const size_t index) {
  switch (index) {
  case 0:
    return x_;
    break;
  case 1:
    return y_;
    break;
  case 2:
    return z_;
    break;
  default:
    throw std::invalid_argument{"Index out of bounds"};
    break;
  }
  // The code will never get to this point
  return x_;
}

Vector3 Vector3::operator+(const Vector3 &rhs) const {
  Vector3 aux{*this};
  aux += rhs;
  return aux;
}

Vector3 Vector3::operator-(const Vector3 &rhs) const {
  Vector3 aux{*this};
  aux -= rhs;
  return aux;
}

Vector3 &Vector3::operator+=(const Vector3 &rhs) {
  x_ += rhs.x_;
  y_ += rhs.y_;
  z_ += rhs.z_;
  return *this;
}

Vector3 &Vector3::operator-=(const Vector3 &rhs) {
  x_ -= rhs.x_;
  y_ -= rhs.y_;
  z_ -= rhs.z_;
  return *this;
}

Vector3 Vector3::operator*(const double rhs) const {
  Vector3 aux{*this};
  aux *= rhs;
  return aux;
}

Vector3 Vector3::operator/(const double rhs) const {
  Vector3 aux{*this};
  aux /= rhs;
  return aux;
}

Vector3 &Vector3::operator*=(const double rhs) {
  x_ *= rhs;
  y_ *= rhs;
  z_ *= rhs;
  return *this;
}

Vector3 &Vector3::operator/=(const double rhs) {
  x_ /= rhs;
  y_ /= rhs;
  z_ /= rhs;
  return *this;
}

Vector3 operator*(const double lhs, const Vector3 &rhs) { return rhs * lhs; }

std::ostream &operator<<(std::ostream &os, const Vector3 &v) {
  return os << "[ " << v[0] << " " << v[1] << " " << v[2] << " ]";
}

} // namespace tijcore
