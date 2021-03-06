/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// Standard library
#include <initializer_list>
#include <memory>

// gtest
#include "gtest/gtest.h"

// tijcore
#include "utils/test_utils.hpp"
#include <tijcore/perception/StaticFrameTransformer.hpp>

namespace tijcore {

namespace test {

namespace {

using ::testing::Test;

class StaticFrameTransformerTests : public Test {
protected:
  const double position_tolerance_{1e-6};
  const double angular_tolerance_{1e-6};
};

class ValidRequestsStaticFrameTransformerTests
    : public StaticFrameTransformerTests {
protected:
  std::unique_ptr<StaticFrameTransformer> uut_;

  ValidRequestsStaticFrameTransformerTests() {
    std::initializer_list<StaticFrameTransformer::TransformTreeLink> links = {
        {"table", RelativePose3{"world", Position::fromVector(0, 0, 1),
                                Rotation::Identity}},
        {"lamp",
         RelativePose3{"table", Position::fromVector(0, 0, 0.1),
                       Rotation{Quaternion(0, 0, -0.3826834, 0.9238795)}}},
        {"book",
         RelativePose3{"table", Position::fromVector(-0.3, 0, 0.1),
                       Rotation{Quaternion(0, 0, 0.3826834, 0.9238795)}}},
    };
    uut_ = std::make_unique<StaticFrameTransformer>(links);
  }
};

TEST_F(ValidRequestsStaticFrameTransformerTests, TrivialWorldToWorld) {
  auto result = uut_->transformPoseToFrame(RelativePose3{"world"}, "world");
  auto expected = RelativePose3{"world", Position::Zero, Rotation::Identity};
  ASSERT_TRUE(RelativePose3::sameRelativePose3(
      expected, result, position_tolerance_, angular_tolerance_));
}

TEST_F(ValidRequestsStaticFrameTransformerTests, TrivialLampToLamp) {
  auto result = uut_->transformPoseToFrame(RelativePose3{"lamp"}, "lamp");
  auto expected = RelativePose3{"lamp", Position::Zero, Rotation::Identity};
  ASSERT_TRUE(RelativePose3::sameRelativePose3(
      expected, result, position_tolerance_, angular_tolerance_));
}

TEST_F(ValidRequestsStaticFrameTransformerTests,
       RegularFrameConversionLampToWorld) {
  {
    auto result = uut_->transformPoseToFrame(RelativePose3{"lamp"}, "world");
    auto expected =
        RelativePose3{"world", Position::fromVector(0, 0, 1.1),
                      Rotation::fromQuaternion(0, 0, -0.3826834, 0.9238795)};
    ASSERT_TRUE(RelativePose3::sameRelativePose3(
        expected, result, position_tolerance_, angular_tolerance_));
  }
  {
    auto result = uut_->transformPoseToFrame(RelativePose3{"world"}, "lamp");
    auto expected =
        RelativePose3{"lamp", Position::fromVector(0, 0, -1.1),
                      Rotation::fromQuaternion(0, 0, 0.3826834, 0.9238795)};
    ASSERT_TRUE(RelativePose3::sameRelativePose3(
        expected, result, position_tolerance_, angular_tolerance_));
  }
}

TEST_F(ValidRequestsStaticFrameTransformerTests,
       RegularFrameConversionBookToWorld) {
  {
    auto result = uut_->transformPoseToFrame(RelativePose3{"book"}, "world");
    auto expected =
        RelativePose3{"world", Position::fromVector(-0.3, 0, 1.1),
                      Rotation::fromQuaternion(0, 0, 0.3826834, 0.9238795)};
    ASSERT_TRUE(RelativePose3::sameRelativePose3(
        expected, result, position_tolerance_, angular_tolerance_));
  }
  {
    auto result = uut_->transformPoseToFrame(RelativePose3{"world"}, "book");
    auto expected = RelativePose3{
        "book",
        Position::fromVector(0.21213203435596423, -0.21213203435596423, -1.1),
        Rotation::fromQuaternion(0, 0, -0.3826834, 0.9238795)};
    ASSERT_TRUE(RelativePose3::sameRelativePose3(
        expected, result, position_tolerance_, angular_tolerance_));
  }
}

TEST_F(ValidRequestsStaticFrameTransformerTests,
       RegularFrameConversionBookToWorldLamp) {
  auto result = uut_->transformPoseToFrame(RelativePose3{"book"}, "lamp");
  auto expected = RelativePose3{
      "lamp",
      Position::fromVector(-0.21213203435596423, -0.21213203435596423, 0),
      Rotation::fromQuaternion(0, 0, 0.7071068, 0.7071068)};
  ASSERT_TRUE(RelativePose3::sameRelativePose3(
      expected, result, position_tolerance_, angular_tolerance_));
}

TEST_F(ValidRequestsStaticFrameTransformerTests,
       InvalidFrameTransformationsThrwo) {
  ASSERT_THROW(uut_->transformPoseToFrame(RelativePose3{"world"}, "foo"),
               std::invalid_argument);
  ASSERT_THROW(uut_->transformPoseToFrame(RelativePose3{"foo"}, "world"),
               std::invalid_argument);
}

class InvalidRequestsStaticFrameTransformerTests
    : public StaticFrameTransformerTests {};

TEST_F(InvalidRequestsStaticFrameTransformerTests,
       LinkWithMissingParentThrows) {
  std::initializer_list<StaticFrameTransformer::TransformTreeLink> links = {
      {"table", RelativePose3{"world", Position::fromVector(0, 0, 1),
                              Rotation::Identity}},
      {"lamp",
       RelativePose3{"table", Position::fromVector(0, 0, 0.1),
                     Rotation{Quaternion(0, 0, -0.3826834, 0.9238795)}}},
      {"book", RelativePose3{"table", Position::fromVector(-0.3, 0, 0.1),
                             Rotation{Quaternion(0, 0, 0.3826834, 0.9238795)}}},
      {"tv", RelativePose3{"remote", Position::fromVector(-0.3, 0, 0.1),
                           Rotation{Quaternion(0, 0, 0.3826834, 0.9238795)}}},
  };
  ASSERT_THROW(std::make_unique<StaticFrameTransformer>(links),
               std::invalid_argument);
}

TEST_F(InvalidRequestsStaticFrameTransformerTests,
       LinkWithIndirectMissingdParentThrows) {
  std::initializer_list<StaticFrameTransformer::TransformTreeLink> links = {
      {"table", RelativePose3{"world", Position::fromVector(0, 0, 1),
                              Rotation::Identity}},
      {"lamp",
       RelativePose3{"table", Position::fromVector(0, 0, 0.1),
                     Rotation{Quaternion(0, 0, -0.3826834, 0.9238795)}}},
      {"book", RelativePose3{"table", Position::fromVector(-0.3, 0, 0.1),
                             Rotation{Quaternion(0, 0, 0.3826834, 0.9238795)}}},
      {"tv", RelativePose3{"remote", Position::Zero, Rotation::Identity}},
      {"remote", RelativePose3{"window", Position::Zero, Rotation::Identity}},
  };
  ASSERT_THROW(std::make_unique<StaticFrameTransformer>(links),
               std::invalid_argument);
}

TEST_F(InvalidRequestsStaticFrameTransformerTests,
       LinkWithRecursivePathThrows) {
  std::initializer_list<StaticFrameTransformer::TransformTreeLink> links = {
      {"table", RelativePose3{"world", Position::fromVector(0, 0, 1),
                              Rotation::Identity}},
      {"lamp",
       RelativePose3{"table", Position::fromVector(0, 0, 0.1),
                     Rotation{Quaternion(0, 0, -0.3826834, 0.9238795)}}},
      {"book", RelativePose3{"table", Position::fromVector(-0.3, 0, 0.1),
                             Rotation{Quaternion(0, 0, 0.3826834, 0.9238795)}}},
      {"tv", RelativePose3{"tv", Position::Zero, Rotation::Identity}},
  };
  ASSERT_THROW(std::make_unique<StaticFrameTransformer>(links),
               std::invalid_argument);
}

TEST_F(InvalidRequestsStaticFrameTransformerTests,
       LinkWithIndirectRecursivePathThrows) {
  std::initializer_list<StaticFrameTransformer::TransformTreeLink> links = {
      {"table", RelativePose3{"world", Position::fromVector(0, 0, 1),
                              Rotation::Identity}},
      {"lamp",
       RelativePose3{"table", Position::fromVector(0, 0, 0.1),
                     Rotation{Quaternion(0, 0, -0.3826834, 0.9238795)}}},
      {"book", RelativePose3{"table", Position::fromVector(-0.3, 0, 0.1),
                             Rotation{Quaternion(0, 0, 0.3826834, 0.9238795)}}},
      {"tv", RelativePose3{"tv", Position::Zero, Rotation::Identity}},
      {"remote", RelativePose3{"tv", Position::Zero, Rotation::Identity}},
  };
  ASSERT_THROW(std::make_unique<StaticFrameTransformer>(links),
               std::invalid_argument);
}

} // namespace

} // namespace test

} // namespace tijcore
