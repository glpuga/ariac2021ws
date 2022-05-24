/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// standard library
#include <memory>

// gtest
#include "gmock/gmock.h"
#include "gtest/gtest.h"

// tijcore
#include <tijcore/resources/SpatialMutualExclusionManager.hpp>
#include <tijcore/utils/StaticFrameTransformer.hpp>

namespace tijcore
{
namespace utils
{
namespace test
{
namespace
{
using ::testing::InSequence;
using ::testing::Return;
using ::testing::Test;

struct SpatialMutualExclusionManagerTests : public Test
{
  std::shared_ptr<StaticFrameTransformer> static_frame_transformer_;

  const tijmath::RelativePose3 robot_1_rel_pose_{ "robot_1",
                                                  tijmath::Position::fromVector(0, 0, 0),
                                                  {} };

  const tijmath::RelativePose3 robot_2_rel_pose_{ "robot_2",
                                                  tijmath::Position::fromVector(0, 0, 0),
                                                  {} };

  const tijmath::RelativePose3 robot_in_between_rel_pose_{ "robot_in_between",
                                                           tijmath::Position::fromVector(0, 0, 0),
                                                           {} };

  const tijmath::RelativePose3 robot_close_but_not_in_between_{ "robot_close_but_not_in_between",
                                                                tijmath::Position::fromVector(0, 0,
                                                                                              0),
                                                                {} };

  SpatialMutualExclusionManagerTests()
  {
    static_frame_transformer_ = std::make_shared<StaticFrameTransformer>(
        std::initializer_list<StaticFrameTransformer::TransformTreeLink>{
            {
                "robot_1",
                tijmath::RelativePose3{ "world", tijmath::Position::fromVector(1, 0, 1),
                                        tijmath::Rotation{ tijmath::Rotation::Identity } },
            },
            {
                "robot_2",
                tijmath::RelativePose3{ "world", tijmath::Position::fromVector(0, 1, 1),
                                        tijmath::Rotation{ tijmath::Rotation::Identity } },
            },
            {
                "robot_in_between",
                tijmath::RelativePose3{ "world", tijmath::Position::fromVector(0.5, 0.5, 1),
                                        tijmath::Rotation{ tijmath::Rotation::Identity } },
            },
            {
                "robot_close_but_not_in_between",
                tijmath::RelativePose3{ "world", tijmath::Position::fromVector(0.0, 0.0, 1),
                                        tijmath::Rotation{ tijmath::Rotation::Identity } },
            },
        });
  }
};

TEST_F(SpatialMutualExclusionManagerTests, ConstructionTest)
{
  SpatialMutualExclusionManager uut("world", static_frame_transformer_);
}

TEST_F(SpatialMutualExclusionManagerTests, NonOverlappingVolumesTest)
{
  SpatialMutualExclusionManager uut("world", static_frame_transformer_);
  const auto v1 = uut.lockSphereVolume(robot_1_rel_pose_, 0.5);
  ASSERT_TRUE(v1.has_value());
  const auto v2 = uut.lockSphereVolume(robot_2_rel_pose_, 0.5);
  ASSERT_TRUE(v2.has_value());
}

TEST_F(SpatialMutualExclusionManagerTests, OverlappingVolumesTest)
{
  SpatialMutualExclusionManager uut("world", static_frame_transformer_);
  const auto v1 = uut.lockSphereVolume(robot_1_rel_pose_, 0.5);
  ASSERT_TRUE(v1.has_value());

  // taking the same volume twice should fail
  {
    // smaller radius
    const auto v2 = uut.lockSphereVolume(robot_1_rel_pose_, 0.01);
    ASSERT_FALSE(v2.has_value());
  }
  {
    // smaller same
    const auto v2 = uut.lockSphereVolume(robot_1_rel_pose_, 0.5);
    ASSERT_FALSE(v2.has_value());
  }
  {
    // larger radius
    const auto v2 = uut.lockSphereVolume(robot_1_rel_pose_, 1.5);
    ASSERT_FALSE(v2.has_value());
  }
  {
    // partial overlap
    const auto v2 = uut.lockSphereVolume(robot_2_rel_pose_, 1.1);
    ASSERT_FALSE(v2.has_value());
  }
}

TEST_F(SpatialMutualExclusionManagerTests, VolumesAreFreedOnRelease)
{
  SpatialMutualExclusionManager uut("world", static_frame_transformer_);
  // lock volume 1
  auto v1 = uut.lockSphereVolume(robot_1_rel_pose_, 0.9);
  ASSERT_TRUE(v1.has_value());
  // try locking volume 2, fails because they overlap
  const auto v2 = uut.lockSphereVolume(robot_2_rel_pose_, 0.9);
  ASSERT_FALSE(v2.has_value());
  // release volume 1
  v1->release();
  // try again volume 2, this time it succeeds
  const auto v3 = uut.lockSphereVolume(robot_2_rel_pose_, 0.9);
  ASSERT_TRUE(v3.has_value());
}

TEST_F(SpatialMutualExclusionManagerTests, VolumesAreFreedOnDestruct)
{
  SpatialMutualExclusionManager uut("world", static_frame_transformer_);
  {  // lock volume 1
    const auto v1 = uut.lockSphereVolume(robot_1_rel_pose_, 0.9);
    ASSERT_TRUE(v1.has_value());
    // try locking volume 2, fails because they overlap
    const auto v2 = uut.lockSphereVolume(robot_2_rel_pose_, 0.9);
    ASSERT_FALSE(v2.has_value());
  }
  // try again volume 2, this time it succeeds because v1 was destroyed
  const auto v3 = uut.lockSphereVolume(robot_2_rel_pose_, 0.9);
  ASSERT_TRUE(v3.has_value());
}

TEST_F(SpatialMutualExclusionManagerTests, LockingAFullPathLocksVolumesInBetween)
{
  SpatialMutualExclusionManager uut("world", static_frame_transformer_);
  {
    // try volumes 1 and 2
    const auto v1 = uut.lockSphereVolume(robot_1_rel_pose_, 0.2);
    const auto v2 = uut.lockSphereVolume(robot_2_rel_pose_, 0.2);
    ASSERT_TRUE(v1.has_value());
    ASSERT_TRUE(v2.has_value());
    // lock the volume in between. It'll succeed because all volumes are mutally exclusive
    const auto v3 = uut.lockSphereVolume(robot_in_between_rel_pose_, 0.2);
    ASSERT_TRUE(v3.has_value());
  }
  {
    // try locking volume between poses 1 and 2
    const auto v1 = uut.lockSpheresPathVolume(robot_1_rel_pose_, robot_2_rel_pose_, 0.2);
    ASSERT_TRUE(v1.has_value());
    // lock the volume in between. It'll fail, because all the path between 1 and 2 is taken
    const auto v3 = uut.lockSphereVolume(robot_in_between_rel_pose_, 0.2);
    ASSERT_FALSE(v3.has_value());
    // a separate volume away from the main path will still lock just fine
    const auto v4 = uut.lockSphereVolume(robot_close_but_not_in_between_, 0.2);
    ASSERT_TRUE(v4.has_value());
  }
  {
    // after the path has been released, the volume in between is available again
    const auto v2 = uut.lockSphereVolume(robot_in_between_rel_pose_, 0.2);
    ASSERT_TRUE(v2.has_value());
  }
}

TEST_F(SpatialMutualExclusionManagerTests, LockingAFullPathWorksIfBothEndsOverlap)
{
  SpatialMutualExclusionManager uut("world", static_frame_transformer_);
  // try locking volume between poses 1 and 2. The radii overlap, but it should succeed anyway
  const auto v1 = uut.lockSpheresPathVolume(robot_1_rel_pose_, robot_2_rel_pose_, 3.0);
  ASSERT_TRUE(v1.has_value());
}

}  // namespace

}  // namespace test

}  // namespace utils

}  // namespace tijcore
