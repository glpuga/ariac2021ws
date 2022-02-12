/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// standard library
#include <utility>

// Standard library
#include <cmath>

// gtest
#include "gtest/gtest.h"

// tijmath
#include <tijmath/Isometry.hpp>

#include "utils/test_utils.hpp"

namespace tijmath
{
namespace test
{
namespace
{
using ::testing::Test;

class IsometryTests : public Test
{
protected:
  const double tolerance_{ 1e-6 };
};

TEST_F(IsometryTests, DefaultConstructor)
{
  Isometry uut;
  ASSERT_TRUE(areEqualWithinTolerance(uut.rotation(), Matrix3::Identity, tolerance_));
  ASSERT_TRUE(areEqualWithinTolerance(uut.translation(), Vector3::Zero, tolerance_));
}

TEST_F(IsometryTests, RotTranConstructor)
{
  const Matrix3 rotation{ Vector3{ 0, 1, 0 }, Vector3{ 0, 0, 1 }, Vector3{ 1, 0, 0 } };
  const Vector3 translation{ 2, 3, 4 };
  Isometry uut{ rotation, translation };
  ASSERT_TRUE(areEqualWithinTolerance(uut.rotation(), rotation, tolerance_));
  ASSERT_TRUE(areEqualWithinTolerance(uut.translation(), translation, tolerance_));
}

TEST_F(IsometryTests, StaticConstants)
{
  auto uut = Isometry::Identity;
  ASSERT_TRUE(areEqualWithinTolerance(Matrix3::Identity, uut.rotation(), tolerance_));
  ASSERT_TRUE(areEqualWithinTolerance(Vector3::Zero, uut.translation(), tolerance_));
}

TEST_F(IsometryTests, CopyConstructor)
{
  const Matrix3 rotation{ Vector3{ 0, 1, 0 }, Vector3{ 0, 0, 1 }, Vector3{ 1, 0, 0 } };
  const Vector3 translation{ 2, 3, 4 };
  Isometry org{ rotation, translation };
  const Isometry uut{ org };
  ASSERT_TRUE(areEqualWithinTolerance(org, uut, tolerance_));
}

TEST_F(IsometryTests, MoveConstructor)
{
  const Matrix3 rotation{ Vector3{ 0, 1, 0 }, Vector3{ 0, 0, 1 }, Vector3{ 1, 0, 0 } };
  const Vector3 translation{ 2, 3, 4 };
  Isometry org{ rotation, translation };
  const Isometry org_copy{ org };
  const Isometry uut{ std::move(org) };
  ASSERT_TRUE(areEqualWithinTolerance(org, uut, tolerance_));
}

TEST_F(IsometryTests, CopyAssigment)
{
  const Matrix3 rotation{ Vector3{ 0, 1, 0 }, Vector3{ 0, 0, 1 }, Vector3{ 1, 0, 0 } };
  const Vector3 translation{ 2, 3, 4 };
  Isometry org{ rotation, translation };
  Isometry uut;
  ASSERT_TRUE(areEqualWithinTolerance(Isometry::Identity, uut, tolerance_));
  uut = org;
  ASSERT_FALSE(areEqualWithinTolerance(Isometry::Identity, uut, tolerance_));
  ASSERT_TRUE(areEqualWithinTolerance(org, uut, tolerance_));
}

TEST_F(IsometryTests, MoveAssingment)
{
  const Matrix3 rotation{ Vector3{ 0, 1, 0 }, Vector3{ 0, 0, 1 }, Vector3{ 1, 0, 0 } };
  const Vector3 translation{ 2, 3, 4 };
  Isometry org{ rotation, translation };
  const Isometry org_copy{ org };
  Isometry uut;
  ASSERT_TRUE(areEqualWithinTolerance(Isometry::Identity, uut, tolerance_));
  uut = std::move(org);
  ASSERT_FALSE(areEqualWithinTolerance(Isometry::Identity, uut, tolerance_));
  ASSERT_TRUE(areEqualWithinTolerance(org, uut, tolerance_));
}

TEST_F(IsometryTests, Translation)
{
  {
    // const version
    const Vector3 translation{ 2, 3, 4 };
    const Isometry uut{ Matrix3::Identity, translation };
    ASSERT_TRUE(areEqualWithinTolerance(translation, uut.translation(), tolerance_));
  }
  {
    // non-const version
    const Vector3 translation1{ 2, 3, 4 };
    const Vector3 translation2{ 3, 4, 5 };
    Isometry uut{ Matrix3::Identity, translation1 };
    ASSERT_TRUE(areEqualWithinTolerance(translation1, uut.translation(), tolerance_));
    uut.translation() = translation2;
    ASSERT_FALSE(areEqualWithinTolerance(translation1, uut.translation(), tolerance_));
    ASSERT_TRUE(areEqualWithinTolerance(translation2, uut.translation(), tolerance_));
  }
}

TEST_F(IsometryTests, Rotation)
{
  {
    // const version
    const Matrix3 rotation1{ Vector3{ 0, 1, 0 }, Vector3{ 0, 0, 1 }, Vector3{ 1, 0, 0 } };
    Isometry uut{ rotation1, Vector3::Zero };
    ASSERT_TRUE(areEqualWithinTolerance(rotation1, uut.rotation(), tolerance_));
  }
  {
    // non-const version
    const Matrix3 rotation1{ Vector3{ 0, 1, 0 }, Vector3{ 0, 0, 1 }, Vector3{ 1, 0, 0 } };
    const Matrix3 rotation2{ Vector3{ 0, 0, 1 }, Vector3{ 1, 0, 0 }, Vector3{ 0, 1, 0 } };
    Isometry uut{ rotation1, Vector3::Zero };
    ASSERT_TRUE(areEqualWithinTolerance(rotation1, uut.rotation(), tolerance_));
    uut.rotation() = rotation2;
    ASSERT_FALSE(areEqualWithinTolerance(rotation1, uut.rotation(), tolerance_));
    ASSERT_TRUE(areEqualWithinTolerance(rotation2, uut.rotation(), tolerance_));
  }
}

TEST_F(IsometryTests, Composition)
{
  {
    const Matrix3 rotation1{ Vector3{ 0, 1, 0 }, Vector3{ 0, 0, 1 }, Vector3{ 1, 0, 0 } };
    const Vector3 translation1{ 1, 2, 3 };
    const Matrix3 rotation2{ Vector3{ 0, 0, 1 }, Vector3{ 1, 0, 0 }, Vector3{ 0, 1, 0 } };
    const Vector3 translation2{ 10, 20, 30 };
    const Isometry op1{ rotation1, translation1 };
    const Isometry op2{ rotation2, translation2 };
    const auto uut = op1 * op2;
    ASSERT_TRUE(areEqualWithinTolerance(rotation1 * rotation2, uut.rotation(), tolerance_));
    ASSERT_TRUE(areEqualWithinTolerance(rotation1 * translation2 + translation1, uut.translation(), tolerance_));
  }
  {
    const Matrix3 rotation1{ Vector3{ 0, 1, 0 }, Vector3{ 0, 0, 1 }, Vector3{ 1, 0, 0 } };
    const Vector3 translation1{ 1, 2, 3 };
    const Matrix3 rotation2{ Vector3{ 0, 0, 1 }, Vector3{ 1, 0, 0 }, Vector3{ 0, 1, 0 } };
    const Vector3 translation2{ 10, 20, 30 };

    const Isometry op2{ rotation2, translation2 };
    Isometry uut{ rotation1, translation1 };
    uut *= op2;
    ASSERT_TRUE(areEqualWithinTolerance(rotation1 * rotation2, uut.rotation(), tolerance_));
    ASSERT_TRUE(areEqualWithinTolerance(rotation1 * translation2 + translation1, uut.translation(), tolerance_));
  }
}

TEST_F(IsometryTests, InverseIsometry)
{
  const Matrix3 rotation1{ Vector3{ 0, 1, 0 }, Vector3{ 0, 0, 1 }, Vector3{ 1, 0, 0 } };
  const Vector3 translation1{ 1, 2, 3 };
  const Isometry uut{ rotation1, translation1 };
  ASSERT_TRUE(areEqualWithinTolerance(rotation1.inv(), uut.inv().rotation(), tolerance_));
  ASSERT_TRUE(areEqualWithinTolerance((-1) * rotation1.inv() * translation1, uut.inv().translation(), tolerance_));
  ASSERT_TRUE(areEqualWithinTolerance(Isometry::Identity, uut * uut.inv(), tolerance_));
  ASSERT_TRUE(areEqualWithinTolerance(Isometry::Identity, uut.inv() * uut, tolerance_));
}

}  // namespace

}  // namespace test

}  // namespace tijmath
