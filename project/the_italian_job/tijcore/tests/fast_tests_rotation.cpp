/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// Standard library
#include <iostream>
#include <sstream>

// gtest
#include "gtest/gtest.h"

// tijcore
#include "utils/test_utils.hpp"
#include <tijcore/math/Rotation.hpp>

namespace tijcore {

namespace test {

namespace {

using ::testing::Test;

class RotationTests : public Test {
protected:
  const double tolerance_{1e-6};
};

TEST_F(RotationTests, DefaultConstructor) {
  Rotation uut;
  ASSERT_NEAR(0, uut.quaternion().x(), tolerance_);
  ASSERT_NEAR(0, uut.quaternion().y(), tolerance_);
  ASSERT_NEAR(0, uut.quaternion().z(), tolerance_);
  ASSERT_NEAR(1, uut.quaternion().w(), tolerance_);
}

TEST_F(RotationTests, QuaternionConstructor) {
  {
    Rotation uut{Quaternion{0, 0, 0, 1}};
    ASSERT_NEAR(0, uut.quaternion().x(), tolerance_);
    ASSERT_NEAR(0, uut.quaternion().y(), tolerance_);
    ASSERT_NEAR(0, uut.quaternion().z(), tolerance_);
    ASSERT_NEAR(1, uut.quaternion().w(), tolerance_);
  }
  {
    Rotation uut{Quaternion{0, 0, 1, 0}};
    ASSERT_NEAR(0, uut.quaternion().x(), tolerance_);
    ASSERT_NEAR(0, uut.quaternion().y(), tolerance_);
    ASSERT_NEAR(1, uut.quaternion().z(), tolerance_);
    ASSERT_NEAR(0, uut.quaternion().w(), tolerance_);
  }
  {
    Rotation uut{Quaternion{0, 1, 0, 0}};
    ASSERT_NEAR(0, uut.quaternion().x(), tolerance_);
    ASSERT_NEAR(1, uut.quaternion().y(), tolerance_);
    ASSERT_NEAR(0, uut.quaternion().z(), tolerance_);
    ASSERT_NEAR(0, uut.quaternion().w(), tolerance_);
  }
  {
    Rotation uut{Quaternion{1, 0, 0, 0}};
    ASSERT_NEAR(1, uut.quaternion().x(), tolerance_);
    ASSERT_NEAR(0, uut.quaternion().y(), tolerance_);
    ASSERT_NEAR(0, uut.quaternion().z(), tolerance_);
    ASSERT_NEAR(0, uut.quaternion().w(), tolerance_);
  }
}

TEST_F(RotationTests, RotationMatrixConstructor) {
  {
    Rotation uut{Matrix3{Vector3::UnitX, Vector3::UnitY, Vector3::UnitZ}};
    ASSERT_TRUE(areEqualWithinTolerance(
        Vector3::UnitX, uut.rotationMatrix().row(0), tolerance_));
    ASSERT_TRUE(areEqualWithinTolerance(
        Vector3::UnitY, uut.rotationMatrix().row(1), tolerance_));
    ASSERT_TRUE(areEqualWithinTolerance(
        Vector3::UnitZ, uut.rotationMatrix().row(2), tolerance_));
  }
  {
    Rotation uut{Matrix3{Vector3::UnitZ, Vector3::UnitX, Vector3::UnitY}};
    ASSERT_TRUE(areEqualWithinTolerance(
        Vector3::UnitZ, uut.rotationMatrix().row(0), tolerance_));
    ASSERT_TRUE(areEqualWithinTolerance(
        Vector3::UnitX, uut.rotationMatrix().row(1), tolerance_));
    ASSERT_TRUE(areEqualWithinTolerance(
        Vector3::UnitY, uut.rotationMatrix().row(2), tolerance_));
  }
  {
    Rotation uut{Matrix3{Vector3::UnitY, Vector3::UnitZ, Vector3::UnitX}};
    ASSERT_TRUE(areEqualWithinTolerance(
        Vector3::UnitY, uut.rotationMatrix().row(0), tolerance_));
    ASSERT_TRUE(areEqualWithinTolerance(
        Vector3::UnitZ, uut.rotationMatrix().row(1), tolerance_));
    ASSERT_TRUE(areEqualWithinTolerance(
        Vector3::UnitX, uut.rotationMatrix().row(2), tolerance_));
  }
}

} // namespace

} // namespace test

} // namespace tijcore
