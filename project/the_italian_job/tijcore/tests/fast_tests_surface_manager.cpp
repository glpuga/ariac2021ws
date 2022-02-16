/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// gtest
#include "gtest/gtest.h"

// tijcore
#include <tijcore/resources/SurfaceManager.hpp>

namespace tijcore
{
namespace test
{
namespace
{
using ::testing::Test;

class SurfaceManagerTests : public Test
{
public:
};

TEST_F(SurfaceManagerTests, TestOccupancy)
{
  SurfaceManager uut{ -1, -2, 3, 6 };
  ASSERT_TRUE(uut.regionIsAvailable(0, -1, 0.9));
  ASSERT_TRUE(uut.regionIsAvailable(1, -1, 0.9));
  ASSERT_TRUE(uut.regionIsAvailable(0, 0, 0.9));
  ASSERT_TRUE(uut.regionIsAvailable(1, 0, 0.9));
  ASSERT_TRUE(uut.regionIsAvailable(0, 1, 0.9));
  ASSERT_TRUE(uut.regionIsAvailable(1, 1, 0.9));
  ASSERT_TRUE(uut.regionIsAvailable(0, 2, 0.9));
  ASSERT_TRUE(uut.regionIsAvailable(1, 2, 0.9));
  ASSERT_TRUE(uut.regionIsAvailable(0, 3, 0.9));
  ASSERT_TRUE(uut.regionIsAvailable(1, 3, 0.9));

  uut.markAsOccupied(0, -1, 1);
  uut.markAsOccupied(1, 3, 1);

  ASSERT_FALSE(uut.regionIsAvailable(0, -1, 0.9));
  ASSERT_FALSE(uut.regionIsAvailable(1, -1, 0.9));
  ASSERT_FALSE(uut.regionIsAvailable(0, 0, 0.9));
  ASSERT_FALSE(uut.regionIsAvailable(1, 0, 0.9));
  ASSERT_TRUE(uut.regionIsAvailable(0, 1, 0.9));
  ASSERT_TRUE(uut.regionIsAvailable(1, 1, 0.9));
  ASSERT_FALSE(uut.regionIsAvailable(0, 2, 0.9));
  ASSERT_FALSE(uut.regionIsAvailable(1, 2, 0.9));
  ASSERT_FALSE(uut.regionIsAvailable(0, 3, 0.9));
  ASSERT_FALSE(uut.regionIsAvailable(1, 3, 0.9));
}

TEST_F(SurfaceManagerTests, TestIsWithinSurface)
{
  SurfaceManager uut{ -1, -2, 3, 6 };

  ASSERT_TRUE(uut.regionIsWithinSurface(0.01, 0.0, 1.0));
  ASSERT_TRUE(uut.regionIsWithinSurface(0.0, 0.99, 1.0));
  ASSERT_TRUE(uut.regionIsWithinSurface(0.99, 0.0, 1.0));
  ASSERT_TRUE(uut.regionIsWithinSurface(0.0, 2.99, 1.0));

  ASSERT_FALSE(uut.regionIsWithinSurface(-0.01, 0.0, 1.0));
  ASSERT_FALSE(uut.regionIsWithinSurface(0.0, -1.01, 1.0));
  ASSERT_FALSE(uut.regionIsWithinSurface(1.01, 0.0, 1.0));
  ASSERT_FALSE(uut.regionIsWithinSurface(0.0, 3.01, 1.0));
}

TEST_F(SurfaceManagerTests, TestFindFreeRegion)
{
  const double random_pose_tolerance_{ 0.2 };
  {
    SurfaceManager uut{ -2, -3, 4, 6 };
    uut.markAsOccupied(-1, -2, 1);
    uut.markAsOccupied(1, -2, 1);
    auto opt_pos = uut.findFreeRegion(1.0);

    ASSERT_TRUE(opt_pos);
    double x, y;
    std::tie(x, y) = opt_pos.value();
    ASSERT_NEAR(0.0, x, random_pose_tolerance_);
    ASSERT_NEAR(2.0, y, random_pose_tolerance_);
  }

  {
    SurfaceManager uut{ -2, -3, 4, 6 };
    uut.markAsOccupied(-1, 2, 1);
    uut.markAsOccupied(1, 2, 1);
    auto opt_pos = uut.findFreeRegion(1.0);

    ASSERT_TRUE(opt_pos);
    double x, y;
    std::tie(x, y) = opt_pos.value();
    ASSERT_NEAR(0.0, x, random_pose_tolerance_);
    ASSERT_NEAR(-2.0, y, random_pose_tolerance_);
  }

  {
    SurfaceManager uut{ -2, -3, 4, 6 };
    uut.markAsOccupied(-1, -2, 1);
    uut.markAsOccupied(-1, 2, 1);
    auto opt_pos = uut.findFreeRegion(1.0);

    ASSERT_TRUE(opt_pos);
    double x, y;
    std::tie(x, y) = opt_pos.value();
    ASSERT_NEAR(1.0, x, random_pose_tolerance_);
    ASSERT_NEAR(0.0, y, random_pose_tolerance_);
  }

  {
    SurfaceManager uut{ -2, -3, 4, 6 };
    uut.markAsOccupied(1, -2, 1);
    uut.markAsOccupied(1, 2, 1);
    auto opt_pos = uut.findFreeRegion(1.0);

    ASSERT_TRUE(opt_pos);
    double x, y;
    std::tie(x, y) = opt_pos.value();
    ASSERT_NEAR(-1.0, x, random_pose_tolerance_);
    ASSERT_NEAR(0.0, y, random_pose_tolerance_);
  }
}

TEST_F(SurfaceManagerTests, TestFindFreeRegionEmptySurface)
{
  const double random_pose_tolerance_{ 0.2 };

  SurfaceManager uut{ -2, -3, 4, 6 };
  // in an empty surface, find should return the coordinates closest
  // to the (x0,y0) coordinates of the surface
  auto opt_pos = uut.findFreeRegion(1.0);

  ASSERT_TRUE(opt_pos);
  double x, y;
  std::tie(x, y) = opt_pos.value();
  ASSERT_NEAR(-1.0, x, random_pose_tolerance_);
  ASSERT_NEAR(-2.0, y, random_pose_tolerance_);
}

}  // namespace

}  // namespace test

}  // namespace tijcore
