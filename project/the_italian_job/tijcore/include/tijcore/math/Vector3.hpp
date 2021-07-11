/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <cstdint>
#include <ostream>

namespace tijcore {

class Vector3 {
public:
  static const Vector3 UnitX;
  static const Vector3 UnitY;
  static const Vector3 UnitZ;
  static const Vector3 Zero;

  Vector3() = default;

  Vector3(const double x, const double y, const double z)
      : x_{x}, y_{y}, z_{z} {}

  Vector3(const Vector3 &) = default;
  Vector3(Vector3 &&) = default;
  Vector3 &operator=(const Vector3 &) = default;
  Vector3 &operator=(Vector3 &&) = default;

  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }

  double &x() { return x_; }
  double &y() { return y_; }
  double &z() { return z_; }

  double dot(const Vector3 &rhs) const;
  double norm() const;

  // TODO(glpuga) test this method
  Vector3 cross(const Vector3 &rhs) const;

  double operator[](const std::size_t index) const;
  double &operator[](const std::size_t index);

  Vector3 operator+(const Vector3 &rhs) const;
  Vector3 operator-(const Vector3 &rhs) const;
  Vector3 operator*(const double rhs) const;

  // TODO(glpuga) test this method
  Vector3 operator/(const double rhs) const;

  Vector3 &operator+=(const Vector3 &rhs);
  Vector3 &operator-=(const Vector3 &rhs);
  Vector3 &operator*=(const double rhs);

  // TODO(glpuga) test this method
  Vector3 &operator/=(const double rhs);

private:
  double x_{0.0};
  double y_{0.0};
  double z_{0.0};
};

Vector3 operator*(const double lhs, const Vector3 &rhs);

std::ostream &operator<<(std::ostream &os, const Vector3 &v);

} // namespace tijcore
