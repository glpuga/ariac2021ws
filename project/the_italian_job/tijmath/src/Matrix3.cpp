/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <cmath>
#include <ostream>
#include <stdexcept>

// tijmath
#include <tijmath/Matrix3.hpp>

namespace tijmath
{
const Matrix3 Matrix3::Identity{ { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };

const Matrix3 Matrix3::Zero{ { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };

Matrix3::Matrix3() = default;

Matrix3::Matrix3(const Vector3& row0, const Vector3& row1, const Vector3& row2)
  : rows_{ row0, row1, row2 }
{
}

Matrix3::Matrix3(std::initializer_list<Vector3> rows)
{
  auto it = rows.begin();
  rows_[0] = *it++;
  rows_[1] = *it++;
  rows_[2] = *it;
}

Matrix3::Matrix3(const Matrix3&) = default;

Matrix3::Matrix3(Matrix3&&) = default;

Matrix3& Matrix3::operator=(const Matrix3&) = default;

Matrix3& Matrix3::operator=(Matrix3&&) = default;

Vector3 Matrix3::row(const size_t row_n) const
{
  if (row_n < 0 || row_n > 2)
  {
    throw std::out_of_range{ "bad index number" };
  }
  return rows_[row_n];
}

Vector3 Matrix3::col(const size_t col_n) const
{
  if (col_n < 0 || col_n > 2)
  {
    throw std::out_of_range{ "bad index number" };
  }

  return Vector3{ rows_[0][col_n], rows_[1][col_n], rows_[2][col_n] };
}

double Matrix3::det() const
{
  auto a11 = rows_[0][0];
  auto a12 = rows_[0][1];
  auto a13 = rows_[0][2];

  auto a21 = rows_[1][0];
  auto a22 = rows_[1][1];
  auto a23 = rows_[1][2];

  auto a31 = rows_[2][0];
  auto a32 = rows_[2][1];
  auto a33 = rows_[2][2];

  return a11 * (a22 * a33 - a23 * a32) - a12 * (a21 * a33 - a23 * a31) +
         a13 * (a21 * a32 - a22 * a31);
}

Matrix3 Matrix3::trans() const
{
  return Matrix3{ col(0), col(1), col(2) };
}

Matrix3 Matrix3::inv() const
{
  auto a11 = rows_[0][0];
  auto a12 = rows_[0][1];
  auto a13 = rows_[0][2];
  auto a21 = rows_[1][0];
  auto a22 = rows_[1][1];
  auto a23 = rows_[1][2];
  auto a31 = rows_[2][0];
  auto a32 = rows_[2][1];
  auto a33 = rows_[2][2];

  auto b11 = a22 * a33 - a23 * a32;
  auto b12 = -(a21 * a33 - a23 * a31);
  auto b13 = a21 * a32 - a22 * a31;

  auto b21 = -(a12 * a33 - a13 * a32);
  auto b22 = a11 * a33 - a13 * a31;
  auto b23 = -(a11 * a32 - a12 * a31);

  auto b31 = a12 * a23 - a13 * a22;
  auto b32 = -(a11 * a23 - a13 * a21);
  auto b33 = a11 * a22 - a12 * a21;

  return Matrix3(Vector3{ b11, b12, b13 }, Vector3{ b21, b22, b23 }, Vector3{ b31, b32, b33 })
             .trans() *
         (1.0 / this->det());
}

Vector3& Matrix3::operator[](const size_t row_n)
{
  return rows_[row_n];
}

const Vector3& Matrix3::operator[](const size_t row_n) const
{
  return rows_[row_n];
}

Matrix3 Matrix3::operator+(const Matrix3& rhs) const
{
  Matrix3 aux{ *this };
  aux += rhs;
  return aux;
}

Matrix3 Matrix3::operator-(const Matrix3& rhs) const
{
  Matrix3 aux{ *this };
  aux -= rhs;
  return aux;
}

Matrix3 Matrix3::operator*(const Matrix3& rhs) const
{
  Matrix3 aux{ *this };
  aux *= rhs;
  return aux;
}

Vector3 Matrix3::operator*(const Vector3& rhs) const
{
  return Vector3(rows_[0].dot(rhs), rows_[1].dot(rhs), rows_[2].dot(rhs));
}

Matrix3 Matrix3::operator*(const double rhs) const
{
  Matrix3 aux{ *this };
  aux *= rhs;
  return aux;
}

Matrix3& Matrix3::operator+=(const Matrix3& rhs)
{
  rows_[0] += rhs.rows_[0];
  rows_[1] += rhs.rows_[1];
  rows_[2] += rhs.rows_[2];
  return *this;
}

Matrix3& Matrix3::operator-=(const Matrix3& rhs)
{
  rows_[0] -= rhs.rows_[0];
  rows_[1] -= rhs.rows_[1];
  rows_[2] -= rhs.rows_[2];
  return *this;
}

Matrix3& Matrix3::operator*=(const Matrix3& rhs)
{
  auto b11 = row(0).dot(rhs.col(0));
  auto b12 = row(0).dot(rhs.col(1));
  auto b13 = row(0).dot(rhs.col(2));
  auto b21 = row(1).dot(rhs.col(0));
  auto b22 = row(1).dot(rhs.col(1));
  auto b23 = row(1).dot(rhs.col(2));
  auto b31 = row(2).dot(rhs.col(0));
  auto b32 = row(2).dot(rhs.col(1));
  auto b33 = row(2).dot(rhs.col(2));
  *this = Matrix3(Vector3{ b11, b12, b13 }, Vector3{ b21, b22, b23 }, Vector3{ b31, b32, b33 });
  return *this;
}

Matrix3& Matrix3::operator*=(const double rhs)
{
  rows_[0] *= rhs;
  rows_[1] *= rhs;
  rows_[2] *= rhs;
  return *this;
}

Matrix3 operator*(const double lhs, const Matrix3& rhs)
{
  return rhs * lhs;
}

std::ostream& operator<<(std::ostream& os, const Matrix3& m)
{
  return os << "[ " << m[0][0] << " " << m[0][1] << " " << m[0][2] << "; " << m[1][0] << " "
            << m[1][1] << " " << m[1][2] << "; " << m[2][0] << " " << m[2][1] << " " << m[2][2]
            << " ]";
}

}  // namespace tijmath
