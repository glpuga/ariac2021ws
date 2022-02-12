/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library
#include <cmath>

// gtest
#include "gtest/gtest.h"

// tijmath
#include <tijmath/Vector3.hpp>

namespace tijmath
{
namespace test
{
namespace
{
using ::testing::Test;

class Vector3Tests : public Test
{
protected:
  const double tolerance_{ 1e-6 };
};

TEST_F(Vector3Tests, DefaultConstructor)
{
  Vector3 uut;
  ASSERT_NEAR(0, uut.x(), tolerance_);
  ASSERT_NEAR(0, uut.y(), tolerance_);
  ASSERT_NEAR(0, uut.z(), tolerance_);
}

TEST_F(Vector3Tests, StaticConstants)
{
  {
    const auto uut = Vector3::UnitX;
    ASSERT_NEAR(1, uut.x(), tolerance_);
    ASSERT_NEAR(0, uut.y(), tolerance_);
    ASSERT_NEAR(0, uut.z(), tolerance_);
  }
  {
    const auto uut = Vector3::UnitY;
    ASSERT_NEAR(0, uut.x(), tolerance_);
    ASSERT_NEAR(1, uut.y(), tolerance_);
    ASSERT_NEAR(0, uut.z(), tolerance_);
  }
  {
    const auto uut = Vector3::UnitZ;
    ASSERT_NEAR(0, uut.x(), tolerance_);
    ASSERT_NEAR(0, uut.y(), tolerance_);
    ASSERT_NEAR(1, uut.z(), tolerance_);
  }
  {
    const auto uut = Vector3::Zero;
    ASSERT_NEAR(0, uut.x(), tolerance_);
    ASSERT_NEAR(0, uut.y(), tolerance_);
    ASSERT_NEAR(0, uut.z(), tolerance_);
  }
}

TEST_F(Vector3Tests, RegularConstructor)
{
  Vector3 uut{ 1, 2, 3 };
  ASSERT_NEAR(1, uut.x(), tolerance_);
  ASSERT_NEAR(2, uut.y(), tolerance_);
  ASSERT_NEAR(3, uut.z(), tolerance_);
}

TEST_F(Vector3Tests, AdditionSubtraction)
{
  const Vector3 v1{ 1, 2, 3 };
  const Vector3 v2{ 10, 20, 30 };

  Vector3 addition;
  addition = v1 + v2;
  ASSERT_NEAR(11, addition.x(), tolerance_);
  ASSERT_NEAR(22, addition.y(), tolerance_);
  ASSERT_NEAR(33, addition.z(), tolerance_);

  Vector3 subtraction;
  subtraction = v1 - v2;
  ASSERT_NEAR(-9, subtraction.x(), tolerance_);
  ASSERT_NEAR(-18, subtraction.y(), tolerance_);
  ASSERT_NEAR(-27, subtraction.z(), tolerance_);
}

TEST_F(Vector3Tests, AdditionSubtractionWithAssignment)
{
  const Vector3 v2{ 10, 20, 30 };

  Vector3 addition{ 1, 2, 3 };
  addition += v2;
  ASSERT_NEAR(11, addition.x(), tolerance_);
  ASSERT_NEAR(22, addition.y(), tolerance_);
  ASSERT_NEAR(33, addition.z(), tolerance_);

  Vector3 subtraction{ 1, 2, 3 };
  subtraction -= v2;
  ASSERT_NEAR(-9, subtraction.x(), tolerance_);
  ASSERT_NEAR(-18, subtraction.y(), tolerance_);
  ASSERT_NEAR(-27, subtraction.z(), tolerance_);
}

TEST_F(Vector3Tests, DotProduct)
{
  const Vector3 v1{ 2, 3, 4 };
  Vector3 v2{ -20, 30, -40 };
  const auto res1 = v1.dot(v2);
  const auto res2 = v2.dot(v1);
  const double expected_dot_product = -40 + 90 - 160;
  ASSERT_NEAR(expected_dot_product, res1, tolerance_);
  ASSERT_NEAR(expected_dot_product, res2, tolerance_);
}

TEST_F(Vector3Tests, Norm)
{
  const Vector3 v1{ -2, -3, -4 };
  const double expected_norm = std::sqrt(2 * 2 + 3 * 3 + 4 * 4);
  ASSERT_NEAR(expected_norm, v1.norm(), tolerance_);
}

}  // namespace

}  // namespace test

}  // namespace tijmath
