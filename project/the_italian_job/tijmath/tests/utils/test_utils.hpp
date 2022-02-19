/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */
#pragma once

// tijmath
#include <tijmath/Isometry.hpp>
#include <tijmath/Matrix3.hpp>
#include <tijmath/Vector3.hpp>

namespace tijmath
{
namespace test
{
inline bool areEqualWithinTolerance(const Vector3& v1, const Vector3& v2, const double tolerance)
{
  auto within_tolerance = [tolerance](const double a, const double b) {
    return std::abs(a - b) < tolerance;
  };
  return within_tolerance(v1[0], v2[0]) && within_tolerance(v1[1], v2[1]) &&
         within_tolerance(v1[2], v2[2]);
}

inline bool areEqualWithinTolerance(const Matrix3& m1, const Matrix3& m2, const double tolerance)
{
  return areEqualWithinTolerance(m1.row(0), m2.row(0), tolerance) &&
         areEqualWithinTolerance(m1.row(1), m2.row(1), tolerance) &&
         areEqualWithinTolerance(m1.row(2), m2.row(2), tolerance);
}

inline bool areEqualWithinTolerance(const Isometry t1, const Isometry& t2, const double tolerance)
{
  return areEqualWithinTolerance(t1.rotation(), t2.rotation(), tolerance) &&
         areEqualWithinTolerance(t1.translation(), t2.translation(), tolerance);
}

}  // namespace test

}  // namespace tijmath
