/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library
#include <cmath>

// gtest
#include "gtest/gtest.h"

// tijcore
#include "utils/test_utils.hpp"
#include <tijcore/math/Matrix3.hpp>

namespace tijcore {

namespace test {

namespace {

using ::testing::Test;

class Matrix3Tests : public Test {
protected:
  const double tolerance_{1e-3};
};

TEST_F(Matrix3Tests, DefaultConstructor) {
  const Matrix3 uut;
  ASSERT_NEAR(0, uut[0][0], tolerance_);
  ASSERT_NEAR(0, uut[0][1], tolerance_);
  ASSERT_NEAR(0, uut[0][2], tolerance_);
  ASSERT_NEAR(0, uut[1][0], tolerance_);
  ASSERT_NEAR(0, uut[1][1], tolerance_);
  ASSERT_NEAR(0, uut[1][2], tolerance_);
  ASSERT_NEAR(0, uut[2][0], tolerance_);
  ASSERT_NEAR(0, uut[2][1], tolerance_);
  ASSERT_NEAR(0, uut[2][2], tolerance_);
}

TEST_F(Matrix3Tests, ThreeVectorConstructor) {
  const Matrix3 uut{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
  ASSERT_NEAR(1, uut[0][0], tolerance_);
  ASSERT_NEAR(2, uut[0][1], tolerance_);
  ASSERT_NEAR(3, uut[0][2], tolerance_);
  ASSERT_NEAR(4, uut[1][0], tolerance_);
  ASSERT_NEAR(5, uut[1][1], tolerance_);
  ASSERT_NEAR(6, uut[1][2], tolerance_);
  ASSERT_NEAR(7, uut[2][0], tolerance_);
  ASSERT_NEAR(8, uut[2][1], tolerance_);
  ASSERT_NEAR(9, uut[2][2], tolerance_);
}

TEST_F(Matrix3Tests, ZeroInitializer) {
  const Matrix3 uut{Matrix3::Zero};
  Matrix3 zero_matrix{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  ASSERT_TRUE(areEqualWithinTolerance(uut, zero_matrix, tolerance_));
}

TEST_F(Matrix3Tests, UnitInitializer) {
  Matrix3 uut{Matrix3::Identity};
  Matrix3 unit_matrix{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  ASSERT_TRUE(areEqualWithinTolerance(uut, unit_matrix, tolerance_));
}

TEST_F(Matrix3Tests, CopyConstructor) {
  const Matrix3 org{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
  Matrix3 uut(org);
  ASSERT_TRUE(areEqualWithinTolerance(org, uut, tolerance_));
}

TEST_F(Matrix3Tests, MoveConstructor) {
  Matrix3 org{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
  const Matrix3 org_copy{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
  Matrix3 uut(std::move(org));
  ASSERT_TRUE(areEqualWithinTolerance(org_copy, uut, tolerance_));
}

TEST_F(Matrix3Tests, CopyAssignment) {
  const Matrix3 org{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
  Matrix3 uut;
  ASSERT_TRUE(areEqualWithinTolerance(Matrix3::Zero, uut, tolerance_));
  uut = org;
  ASSERT_FALSE(areEqualWithinTolerance(Matrix3::Zero, uut, tolerance_));
  ASSERT_TRUE(areEqualWithinTolerance(org, uut, tolerance_));
}

TEST_F(Matrix3Tests, MoveAssignment) {
  Matrix3 org{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
  const Matrix3 org_copy{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
  Matrix3 uut;
  ASSERT_TRUE(areEqualWithinTolerance(Matrix3::Zero, uut, tolerance_));
  uut = std::move(org);
  ASSERT_FALSE(areEqualWithinTolerance(Matrix3::Zero, uut, tolerance_));
  ASSERT_TRUE(areEqualWithinTolerance(org_copy, uut, tolerance_));
}

TEST_F(Matrix3Tests, RowsGetter) {
  const Matrix3 uut{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
  ASSERT_TRUE(
      areEqualWithinTolerance(uut.row(0), Vector3{1, 2, 3}, tolerance_));
  ASSERT_TRUE(
      areEqualWithinTolerance(uut.row(1), Vector3{4, 5, 6}, tolerance_));
  ASSERT_TRUE(
      areEqualWithinTolerance(uut.row(2), Vector3{7, 8, 9}, tolerance_));
  ASSERT_THROW(uut.row(-1), std::out_of_range);
  ASSERT_THROW(uut.row(3), std::out_of_range);
}

TEST_F(Matrix3Tests, ColsGetter) {
  const Matrix3 uut{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
  ASSERT_TRUE(
      areEqualWithinTolerance(uut.col(0), Vector3{1, 4, 7}, tolerance_));
  ASSERT_TRUE(
      areEqualWithinTolerance(uut.col(1), Vector3{2, 5, 8}, tolerance_));
  ASSERT_TRUE(
      areEqualWithinTolerance(uut.col(2), Vector3{3, 6, 9}, tolerance_));
  ASSERT_THROW(uut.col(-1), std::out_of_range);
  ASSERT_THROW(uut.col(3), std::out_of_range);
}

TEST_F(Matrix3Tests, Determinant) {
  {
    const Matrix3 uut{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    ASSERT_NEAR(0, uut.det(), tolerance_);
  }
  {
    const Matrix3 uut{{1, 2, 3}, {0, 5, 6}, {0, 0, 9}};
    ASSERT_NEAR(45, uut.det(), tolerance_);
  }
  {
    const Matrix3 uut{{1, 0, 0}, {4, 5, 0}, {7, 8, 9}};
    ASSERT_NEAR(45, uut.det(), tolerance_);
  }
  {
    const Matrix3 uut{{11, 0, 0}, {0, 12, 0}, {0, 0, 13}};
    ASSERT_NEAR(1716, uut.det(), tolerance_);
  }

  {
    const Matrix3 uut{{1, 2, 3}, {4, 5, 6}, {7, 8, 1}};
    ASSERT_NEAR(24, uut.det(), tolerance_);
  }
}

TEST_F(Matrix3Tests, Inverse) {
  {
    const Matrix3 uut{{1, 2, 3}, {4, 5, 6}, {7, 8, 1}};
    const Matrix3 expected{Vector3{-1.79167, 0.91667, -0.12500},
                           Vector3{1.58333, -0.83333, 0.25000},
                           Vector3{-0.12500, 0.25000, -0.12500}};
    EXPECT_TRUE(areEqualWithinTolerance(expected, uut.inv(), tolerance_));
    EXPECT_TRUE(areEqualWithinTolerance(Matrix3::Identity, uut * uut.inv(),
                                        tolerance_));
    EXPECT_TRUE(areEqualWithinTolerance(Matrix3::Identity, uut.inv() * uut,
                                        tolerance_));
  }
  {
    const Matrix3 uut{{1, 0, 0}, {0, 2, 0}, {0, 0, 4}};
    const Matrix3 expected{Vector3{1, 0, 0}, Vector3{0, 0.5, 0},
                           Vector3{0, 0, 0.25}};
    EXPECT_TRUE(areEqualWithinTolerance(expected, uut.inv(), tolerance_));
    EXPECT_TRUE(areEqualWithinTolerance(Matrix3::Identity, uut * uut.inv(),
                                        tolerance_));
    EXPECT_TRUE(areEqualWithinTolerance(Matrix3::Identity, uut.inv() * uut,
                                        tolerance_));
  }
}

TEST_F(Matrix3Tests, addition) {
  const Matrix3 op2{{10, 20, 30}, {40, 50, 60}, {70, 80, 90}};
  const Matrix3 expected{{11, 22, 33}, {44, 55, 66}, {77, 88, 99}};
  {
    const Matrix3 op1{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    const auto uut = op1 + op2;
    EXPECT_TRUE(areEqualWithinTolerance(expected, uut, tolerance_));
  }
  {
    Matrix3 uut{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    uut += op2;
    EXPECT_TRUE(areEqualWithinTolerance(expected, uut, tolerance_));
  }
}

TEST_F(Matrix3Tests, subtraction) {
  const Matrix3 op2{{-10, -20, -30}, {-40, -50, -60}, {-70, -80, -90}};
  const Matrix3 expected{{11, 22, 33}, {44, 55, 66}, {77, 88, 99}};
  {
    const Matrix3 op1{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    const auto uut = op1 - op2;
    EXPECT_TRUE(areEqualWithinTolerance(expected, uut, tolerance_));
  }
  {
    Matrix3 uut{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    uut -= op2;
    EXPECT_TRUE(areEqualWithinTolerance(expected, uut, tolerance_));
  }
}

TEST_F(Matrix3Tests, ProductDouble) {
  const Matrix3 expected{{2, 4, 6}, {8, 10, 12}, {14, 16, 18}};
  {
    const Matrix3 op1{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    const auto uut1 = op1 * 2;
    const auto uut2 = 2 * op1;
    EXPECT_TRUE(areEqualWithinTolerance(expected, uut1, tolerance_));
    EXPECT_TRUE(areEqualWithinTolerance(expected, uut2, tolerance_));
  }
  {
    Matrix3 uut{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    uut *= 2;
    EXPECT_TRUE(areEqualWithinTolerance(expected, uut, tolerance_));
  }
}

TEST_F(Matrix3Tests, ProductMatrix) {
  const Matrix3 op2{{10, 20, 30}, {40, 50, 60}, {70, 80, 90}};
  const Matrix3 expected{{300, 360, 420}, {660, 810, 960}, {1020, 1260, 1500}};
  {
    const Matrix3 op1{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    const auto uut1 = op1 * op2;
    EXPECT_TRUE(areEqualWithinTolerance(expected, uut1, tolerance_));
  }
  {
    Matrix3 uut{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    uut *= op2;
    EXPECT_TRUE(areEqualWithinTolerance(expected, uut, tolerance_));
  }
}

TEST_F(Matrix3Tests, ProductVector) {
  const Matrix3 op1{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
  const Vector3 op2{10, 20, 30};
  const Vector3 expected{140, 320, 500};
  const auto uut1 = op1 * op2;
  EXPECT_TRUE(areEqualWithinTolerance(expected, uut1, tolerance_));
}

} // namespace

} // namespace test

} // namespace tijcore
