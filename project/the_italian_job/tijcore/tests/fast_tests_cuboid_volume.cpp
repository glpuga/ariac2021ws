/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// gtest
#include "gtest/gtest.h"

// tijcore
#include <tijcore/utils/CuboidVolume.hpp>

namespace tijcore
{
namespace test
{
namespace
{
using ::testing::Test;

class CuboidVolumeTests : public Test
{
protected:
  const double tolerance_{ 1e-6 };

  const tijmath::Vector3 clrb_{ -1, -2, -3 };
  const tijmath::Vector3 culf_{ 1, 2, 3 };
};

TEST_F(CuboidVolumeTests, ConstructionTest)
{
  const CuboidVolume uut{ clrb_, culf_ };

  EXPECT_NEAR(-1, uut.lowerRightBackCorner().vector().x(), tolerance_);
  EXPECT_NEAR(-2, uut.lowerRightBackCorner().vector().y(), tolerance_);
  EXPECT_NEAR(-3, uut.lowerRightBackCorner().vector().z(), tolerance_);

  EXPECT_NEAR(1, uut.upperLeftFrontCorner().vector().x(), tolerance_);
  EXPECT_NEAR(2, uut.upperLeftFrontCorner().vector().y(), tolerance_);
  EXPECT_NEAR(3, uut.upperLeftFrontCorner().vector().z(), tolerance_);
}

TEST_F(CuboidVolumeTests, ContainsTestVector)
{
  const double delta{ 0.1 };
  CuboidVolume uut{ clrb_, culf_ };

  EXPECT_TRUE(uut.contains(clrb_ + tijmath::Vector3(delta, delta, delta)));
  EXPECT_FALSE(uut.contains(clrb_ + tijmath::Vector3(delta, delta, -delta)));
  EXPECT_FALSE(uut.contains(clrb_ + tijmath::Vector3(delta, -delta, delta)));
  EXPECT_FALSE(uut.contains(clrb_ + tijmath::Vector3(delta, -delta, -delta)));
  EXPECT_FALSE(uut.contains(clrb_ + tijmath::Vector3(-delta, delta, delta)));
  EXPECT_FALSE(uut.contains(clrb_ + tijmath::Vector3(-delta, delta, -delta)));
  EXPECT_FALSE(uut.contains(clrb_ + tijmath::Vector3(-delta, -delta, delta)));
  EXPECT_FALSE(uut.contains(clrb_ + tijmath::Vector3(-delta, -delta, -delta)));

  EXPECT_FALSE(uut.contains(culf_ + tijmath::Vector3(delta, delta, delta)));
  EXPECT_FALSE(uut.contains(culf_ + tijmath::Vector3(delta, delta, -delta)));
  EXPECT_FALSE(uut.contains(culf_ + tijmath::Vector3(delta, -delta, delta)));
  EXPECT_FALSE(uut.contains(culf_ + tijmath::Vector3(delta, -delta, -delta)));
  EXPECT_FALSE(uut.contains(culf_ + tijmath::Vector3(-delta, delta, delta)));
  EXPECT_FALSE(uut.contains(culf_ + tijmath::Vector3(-delta, delta, -delta)));
  EXPECT_FALSE(uut.contains(culf_ + tijmath::Vector3(-delta, -delta, delta)));
  EXPECT_TRUE(uut.contains(culf_ + tijmath::Vector3(-delta, -delta, -delta)));
}

}  // namespace

}  // namespace test

}  // namespace tijcore
