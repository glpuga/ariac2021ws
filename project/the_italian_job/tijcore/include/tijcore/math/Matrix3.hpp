/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <array>
#include <cstdint>

// tijcore
#include <tijcore/math/Vector3.hpp>

namespace tijcore
{
class Matrix3
{
public:
  static const Matrix3 Identity;
  static const Matrix3 Zero;

  Matrix3();

  Matrix3(const Vector3& row0, const Vector3& row1, const Vector3& row2);

  Matrix3(std::initializer_list<Vector3> rows);

  Matrix3(const Matrix3&);
  Matrix3(Matrix3&&);
  Matrix3& operator=(const Matrix3&);
  Matrix3& operator=(Matrix3&&);

  Vector3 row(const size_t row_n) const;
  Vector3 col(const size_t col_n) const;

  double det() const;

  Matrix3 trans() const;

  Matrix3 inv() const;

  Vector3& operator[](const size_t row_n);
  const Vector3& operator[](const size_t row_n) const;

  Matrix3 operator+(const Matrix3& rhs) const;
  Matrix3 operator-(const Matrix3& rhs) const;
  Matrix3 operator*(const Matrix3& rhs) const;
  Vector3 operator*(const Vector3& rhs) const;
  Matrix3 operator*(const double rhs) const;

  Matrix3& operator+=(const Matrix3& rhs);
  Matrix3& operator-=(const Matrix3& rhs);
  Matrix3& operator*=(const Matrix3& rhs);
  Matrix3& operator*=(const double rhs);

private:
  std::array<Vector3, 3> rows_;
};

Matrix3 operator*(const double lhs, const Matrix3& rhs);

std::ostream& operator<<(std::ostream& os, const Matrix3& m);

}  // namespace tijcore
