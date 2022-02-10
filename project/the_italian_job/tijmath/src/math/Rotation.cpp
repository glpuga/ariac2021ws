/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library

// Eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// tijmath
#include <tijmath/math/Rotation.hpp>

namespace tijmath
{
const Rotation Rotation::Identity{ Quaternion{ 0, 0, 0, 1 } };

Rotation::Rotation() = default;

Rotation::Rotation(const Quaternion& quaternion) : quaternion_{ quaternion }
{
}

Rotation::Rotation(const Matrix3& rotation_matrix)
{
  // This a non-trivial transformation, so I'm using Eigen on this.
  Eigen::Matrix3d eigen_mat;
  eigen_mat(0, 0) = rotation_matrix[0][0];
  eigen_mat(0, 1) = rotation_matrix[0][1];
  eigen_mat(0, 2) = rotation_matrix[0][2];
  eigen_mat(1, 0) = rotation_matrix[1][0];
  eigen_mat(1, 1) = rotation_matrix[1][1];
  eigen_mat(1, 2) = rotation_matrix[1][2];
  eigen_mat(2, 0) = rotation_matrix[2][0];
  eigen_mat(2, 1) = rotation_matrix[2][1];
  eigen_mat(2, 2) = rotation_matrix[2][2];
  Eigen::Quaterniond eigen_q{ eigen_mat };
  quaternion_ = Quaternion{ eigen_q.x(), eigen_q.y(), eigen_q.z(), eigen_q.w() };
}

Quaternion Rotation::quaternion() const
{
  return quaternion_;
}

Matrix3 Rotation::rotationMatrix() const
{
  double a{ quaternion_.w() };
  double b{ quaternion_.x() };
  double c{ quaternion_.y() };
  double d{ quaternion_.z() };

  auto a11 = a * a + b * b - c * c - d * d;
  auto a12 = 2 * b * c - 2 * a * d;
  auto a13 = 2 * b * d + 2 * a * c;

  auto a21 = 2 * b * c + 2 * a * d;
  auto a22 = a * a - b * b + c * c - d * d;
  auto a23 = 2 * c * d - 2 * a * b;

  auto a31 = 2 * b * d - 2 * a * c;
  auto a32 = 2 * c * d + 2 * a * b;
  auto a33 = a * a - b * b - c * c + d * d;

  return Matrix3{ Vector3{ a11, a12, a13 }, Vector3{ a21, a22, a23 }, Vector3{ a31, a32, a33 } };
}

Rotation Rotation::fromQuaternion(const double x, const double y, const double z, const double w)
{
  return Rotation(Quaternion(x, y, z, w));
}

Rotation Rotation::fromRollPitchYaw(const double roll, const double pitch, const double yaw)
{
  // Abbreviations for the various angular functions
  double cy = std::cos(yaw * 0.5);
  double sy = std::sin(yaw * 0.5);
  double cp = std::cos(pitch * 0.5);
  double sp = std::sin(pitch * 0.5);
  double cr = std::cos(roll * 0.5);
  double sr = std::sin(roll * 0.5);

  return Rotation{ Quaternion{ sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy,
                               cr * cp * cy + sr * sp * sy } };
}

bool Rotation::sameRotation(const Rotation& lhs, const Rotation& rhs, const double tolerance)
{
  // TODO(glpuga) this is completely wrong and just stop gap that should be
  // enough tests

  const auto lhsq = lhs.quaternion();
  const auto rhsq = rhs.quaternion();

  auto is_within_tolerance = [](const double a, const double b, const double tolerance) {
    auto diff_squared = (a - b) * (a - b);
    return diff_squared < tolerance;
  };

  return is_within_tolerance(lhsq.x(), rhsq.x(), tolerance) && is_within_tolerance(lhsq.y(), rhsq.y(), tolerance) &&
         is_within_tolerance(lhsq.z(), rhsq.z(), tolerance) && is_within_tolerance(lhsq.w(), rhsq.w(), tolerance);
}

std::ostream& operator<<(std::ostream& os, const Rotation& r)
{
  os << r.quaternion();
  return os;
}

}  // namespace tijmath
