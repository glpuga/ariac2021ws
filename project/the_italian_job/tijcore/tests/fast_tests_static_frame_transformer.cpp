/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library
#include <initializer_list>
#include <memory>

// gtest
#include "gtest/gtest.h"

// tijcore
#include <tijcore/perception/StaticFrameTransformer.hpp>
#include "utils/test_utils.hpp"

namespace tijcore
{
namespace test
{
namespace
{
using ::testing::Test;

class StaticFrameTransformerTests : public Test
{
protected:
  const double position_tolerance_{ 1e-6 };
  const double angular_tolerance_{ 1e-6 };
};

class ValidRequestsStaticFrameTransformerTests : public StaticFrameTransformerTests
{
protected:
  std::unique_ptr<StaticFrameTransformer> uut_;

  ValidRequestsStaticFrameTransformerTests()
  {
    std::initializer_list<StaticFrameTransformer::TransformTreeLink> links = {
      { "table",
        tijmath::RelativePose3{ "world", tijmath::Position::fromVector(0, 0, 1), tijmath::Rotation::Identity } },
      { "lamp", tijmath::RelativePose3{ "table", tijmath::Position::fromVector(0, 0, 0.1),
                                        tijmath::Rotation{ tijmath::Quaternion(0, 0, -0.3826834, 0.9238795) } } },
      { "book", tijmath::RelativePose3{ "table", tijmath::Position::fromVector(-0.3, 0, 0.1),
                                        tijmath::Rotation{ tijmath::Quaternion(0, 0, 0.3826834, 0.9238795) } } },
    };
    uut_ = std::make_unique<StaticFrameTransformer>(links);
  }
};

TEST_F(ValidRequestsStaticFrameTransformerTests, TrivialWorldToWorld)
{
  auto result = uut_->transformPoseToFrame(tijmath::RelativePose3{ "world" }, "world");
  auto expected = tijmath::RelativePose3{ "world", tijmath::Position::Zero, tijmath::Rotation::Identity };
  ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(expected, result, position_tolerance_, angular_tolerance_));
}

TEST_F(ValidRequestsStaticFrameTransformerTests, TrivialLampToLamp)
{
  auto result = uut_->transformPoseToFrame(tijmath::RelativePose3{ "lamp" }, "lamp");
  auto expected = tijmath::RelativePose3{ "lamp", tijmath::Position::Zero, tijmath::Rotation::Identity };
  ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(expected, result, position_tolerance_, angular_tolerance_));
}

TEST_F(ValidRequestsStaticFrameTransformerTests, RegularFrameConversionLampToWorld)
{
  {
    auto result = uut_->transformPoseToFrame(tijmath::RelativePose3{ "lamp" }, "world");
    auto expected = tijmath::RelativePose3{ "world", tijmath::Position::fromVector(0, 0, 1.1),
                                            tijmath::Rotation::fromQuaternion(0, 0, -0.3826834, 0.9238795) };
    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(expected, result, position_tolerance_, angular_tolerance_));
  }
  {
    auto result = uut_->transformPoseToFrame(tijmath::RelativePose3{ "world" }, "lamp");
    auto expected = tijmath::RelativePose3{ "lamp", tijmath::Position::fromVector(0, 0, -1.1),
                                            tijmath::Rotation::fromQuaternion(0, 0, 0.3826834, 0.9238795) };
    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(expected, result, position_tolerance_, angular_tolerance_));
  }
}

TEST_F(ValidRequestsStaticFrameTransformerTests, RegularFrameConversionBookToWorld)
{
  {
    auto result = uut_->transformPoseToFrame(tijmath::RelativePose3{ "book" }, "world");
    auto expected = tijmath::RelativePose3{ "world", tijmath::Position::fromVector(-0.3, 0, 1.1),
                                            tijmath::Rotation::fromQuaternion(0, 0, 0.3826834, 0.9238795) };
    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(expected, result, position_tolerance_, angular_tolerance_));
  }
  {
    auto result = uut_->transformPoseToFrame(tijmath::RelativePose3{ "world" }, "book");
    auto expected =
        tijmath::RelativePose3{ "book", tijmath::Position::fromVector(0.21213203435596423, -0.21213203435596423, -1.1),
                                tijmath::Rotation::fromQuaternion(0, 0, -0.3826834, 0.9238795) };
    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(expected, result, position_tolerance_, angular_tolerance_));
  }
}

TEST_F(ValidRequestsStaticFrameTransformerTests, RegularFrameConversionBookToWorldLamp)
{
  auto result = uut_->transformPoseToFrame(tijmath::RelativePose3{ "book" }, "lamp");
  auto expected =
      tijmath::RelativePose3{ "lamp", tijmath::Position::fromVector(-0.21213203435596423, -0.21213203435596423, 0),
                              tijmath::Rotation::fromQuaternion(0, 0, 0.7071068, 0.7071068) };
  ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(expected, result, position_tolerance_, angular_tolerance_));
}

TEST_F(ValidRequestsStaticFrameTransformerTests, InvalidFrameTransformationsThrwo)
{
  ASSERT_THROW(uut_->transformPoseToFrame(tijmath::RelativePose3{ "world" }, "foo"), std::invalid_argument);
  ASSERT_THROW(uut_->transformPoseToFrame(tijmath::RelativePose3{ "foo" }, "world"), std::invalid_argument);
}

class InvalidRequestsStaticFrameTransformerTests : public StaticFrameTransformerTests
{
};

TEST_F(InvalidRequestsStaticFrameTransformerTests, LinkWithMissingParentThrows)
{
  std::initializer_list<StaticFrameTransformer::TransformTreeLink> links = {
    { "table", tijmath::RelativePose3{ "world", tijmath::Position::fromVector(0, 0, 1), tijmath::Rotation::Identity } },
    { "lamp", tijmath::RelativePose3{ "table", tijmath::Position::fromVector(0, 0, 0.1),
                                      tijmath::Rotation{ tijmath::Quaternion(0, 0, -0.3826834, 0.9238795) } } },
    { "book", tijmath::RelativePose3{ "table", tijmath::Position::fromVector(-0.3, 0, 0.1),
                                      tijmath::Rotation{ tijmath::Quaternion(0, 0, 0.3826834, 0.9238795) } } },
    { "tv", tijmath::RelativePose3{ "remote", tijmath::Position::fromVector(-0.3, 0, 0.1),
                                    tijmath::Rotation{ tijmath::Quaternion(0, 0, 0.3826834, 0.9238795) } } },
  };
  ASSERT_THROW(std::make_unique<StaticFrameTransformer>(links), std::invalid_argument);
}

TEST_F(InvalidRequestsStaticFrameTransformerTests, LinkWithIndirectMissingdParentThrows)
{
  std::initializer_list<StaticFrameTransformer::TransformTreeLink> links = {
    { "table", tijmath::RelativePose3{ "world", tijmath::Position::fromVector(0, 0, 1), tijmath::Rotation::Identity } },
    { "lamp", tijmath::RelativePose3{ "table", tijmath::Position::fromVector(0, 0, 0.1),
                                      tijmath::Rotation{ tijmath::Quaternion(0, 0, -0.3826834, 0.9238795) } } },
    { "book", tijmath::RelativePose3{ "table", tijmath::Position::fromVector(-0.3, 0, 0.1),
                                      tijmath::Rotation{ tijmath::Quaternion(0, 0, 0.3826834, 0.9238795) } } },
    { "tv", tijmath::RelativePose3{ "remote", tijmath::Position::Zero, tijmath::Rotation::Identity } },
    { "remote", tijmath::RelativePose3{ "window", tijmath::Position::Zero, tijmath::Rotation::Identity } },
  };
  ASSERT_THROW(std::make_unique<StaticFrameTransformer>(links), std::invalid_argument);
}

TEST_F(InvalidRequestsStaticFrameTransformerTests, LinkWithRecursivePathThrows)
{
  std::initializer_list<StaticFrameTransformer::TransformTreeLink> links = {
    { "table", tijmath::RelativePose3{ "world", tijmath::Position::fromVector(0, 0, 1), tijmath::Rotation::Identity } },
    { "lamp", tijmath::RelativePose3{ "table", tijmath::Position::fromVector(0, 0, 0.1),
                                      tijmath::Rotation{ tijmath::Quaternion(0, 0, -0.3826834, 0.9238795) } } },
    { "book", tijmath::RelativePose3{ "table", tijmath::Position::fromVector(-0.3, 0, 0.1),
                                      tijmath::Rotation{ tijmath::Quaternion(0, 0, 0.3826834, 0.9238795) } } },
    { "tv", tijmath::RelativePose3{ "tv", tijmath::Position::Zero, tijmath::Rotation::Identity } },
  };
  ASSERT_THROW(std::make_unique<StaticFrameTransformer>(links), std::invalid_argument);
}

TEST_F(InvalidRequestsStaticFrameTransformerTests, LinkWithIndirectRecursivePathThrows)
{
  std::initializer_list<StaticFrameTransformer::TransformTreeLink> links = {
    { "table", tijmath::RelativePose3{ "world", tijmath::Position::fromVector(0, 0, 1), tijmath::Rotation::Identity } },
    { "lamp", tijmath::RelativePose3{ "table", tijmath::Position::fromVector(0, 0, 0.1),
                                      tijmath::Rotation{ tijmath::Quaternion(0, 0, -0.3826834, 0.9238795) } } },
    { "book", tijmath::RelativePose3{ "table", tijmath::Position::fromVector(-0.3, 0, 0.1),
                                      tijmath::Rotation{ tijmath::Quaternion(0, 0, 0.3826834, 0.9238795) } } },
    { "tv", tijmath::RelativePose3{ "tv", tijmath::Position::Zero, tijmath::Rotation::Identity } },
    { "remote", tijmath::RelativePose3{ "tv", tijmath::Position::Zero, tijmath::Rotation::Identity } },
  };
  ASSERT_THROW(std::make_unique<StaticFrameTransformer>(links), std::invalid_argument);
}

}  // namespace

}  // namespace test

}  // namespace tijcore
