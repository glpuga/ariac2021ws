/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// standard library
#include <string>

// gtest
#include "gtest/gtest.h"

// tijcore
#include <tijcore/resources/ManagedLocus.hpp>

#include "mocks/ModelContainerMock.hpp"

namespace tijcore
{
namespace utils
{
namespace test
{
namespace
{
using ::testing::Test;

class ManagedLocusTests : public Test
{
public:
  const double tolerance_{ 1e-3 };
  const tijmath::RelativePose3 pose_{ "", tijmath::Position{ tijmath::Vector3{ 1., 2., 3. } },
                                      tijmath::Rotation::Identity };
  const PartId part_id_{ PartTypeId::pump, PartColorId::red };
  const std::string parent_{ "parent_name" };

  ManagedLocusTests()
  {
  }
};

TEST_F(ManagedLocusTests, EmptySpaceConstruction)
{
  auto uut = ManagedLocus::CreateEmptyLocus(parent_, pose_);
  ASSERT_TRUE(uut.isEmptyLocus());
  ASSERT_FALSE(uut.isLocusWithPart());
  ASSERT_THROW({ uut.qualifiedPartInfo().part_type; }, std::logic_error);
  ASSERT_NEAR(pose_.position().vector().x(), uut.pose().position().vector().x(), tolerance_);
  ASSERT_EQ(parent_, uut.parentName());
}

TEST_F(ManagedLocusTests, NonEmptySpaceConstructionNonBroken)
{
  auto uut = ManagedLocus::CreatePartLocus(parent_, pose_, part_id_, false);
  ASSERT_FALSE(uut.isEmptyLocus());
  ASSERT_TRUE(uut.isLocusWithPart());
  const auto part_id = uut.qualifiedPartInfo().part_type;
  const auto broken = uut.qualifiedPartInfo().part_is_broken;
  ASSERT_EQ(part_id_, part_id);
  ASSERT_EQ(false, broken);
  ASSERT_NEAR(pose_.position().vector().x(), uut.pose().position().vector().x(), tolerance_);
  ASSERT_EQ(parent_, uut.parentName());
}

TEST_F(ManagedLocusTests, NonEmptySpaceConstructionBroken)
{
  auto uut = ManagedLocus::CreatePartLocus(parent_, pose_, part_id_, true);
  ASSERT_FALSE(uut.isEmptyLocus());
  ASSERT_TRUE(uut.isLocusWithPart());
  const auto part_id = uut.qualifiedPartInfo().part_type;
  const auto broken = uut.qualifiedPartInfo().part_is_broken;
  ASSERT_EQ(part_id_, part_id);
  ASSERT_EQ(true, broken);
  ASSERT_NEAR(pose_.position().vector().x(), uut.pose().position().vector().x(), tolerance_);
  ASSERT_EQ(parent_, uut.parentName());
}

TEST_F(ManagedLocusTests, TransferPartFromHereToThereWorks)
{
  auto is_empty_space = [](const auto& uut) {
    bool retval;
    try
    {
      retval = uut.isEmptyLocus() && !uut.isLocusWithPart();
    }
    catch (...)
    {
      retval = false;
    }
    return retval;
  };

  auto is_occupied_space = [](const auto& uut, const PartId& expected_part_id,
                              const bool expected_broken_value) {
    bool retval;
    try
    {
      const auto part_id = uut.qualifiedPartInfo().part_type;
      const auto broken = uut.qualifiedPartInfo().part_is_broken;
      retval = !uut.isEmptyLocus() && uut.isLocusWithPart() && (expected_part_id == part_id) &&
               (expected_broken_value == broken);
    }
    catch (...)
    {
      retval = false;
    }
    return retval;
  };

  auto is_the_same_pose = [](const tijmath::RelativePose3& expected,
                             const tijmath::RelativePose3& actual, const double tolerance) {
    auto is_within_tolerance = [](const double a, const double b, const double tolerance) {
      return ((a - b) < tolerance) && ((b - a) < tolerance);
    };

    return (expected.frameId() == actual.frameId()) &&
           is_within_tolerance(expected.position().vector().x(), actual.position().vector().x(),
                               tolerance) &&
           is_within_tolerance(expected.position().vector().y(), actual.position().vector().y(),
                               tolerance) &&
           is_within_tolerance(expected.position().vector().z(), actual.position().vector().z(),
                               tolerance) &&
           is_within_tolerance(expected.rotation().quaternion().x(),
                               actual.rotation().quaternion().x(), tolerance) &&
           is_within_tolerance(expected.rotation().quaternion().y(),
                               actual.rotation().quaternion().y(), tolerance) &&
           is_within_tolerance(expected.rotation().quaternion().z(),
                               actual.rotation().quaternion().z(), tolerance) &&
           is_within_tolerance(expected.rotation().quaternion().w(),
                               actual.rotation().quaternion().w(), tolerance);
  };

  auto src_parent{ "src_container" };
  auto dst_parent{ "dst_containar" };

  const tijmath::RelativePose3 src_pose{ "", tijmath::Position{ tijmath::Vector3{ 1., 2., 3. } },
                                         tijmath::Rotation::Identity };
  const tijmath::RelativePose3 dst_pose{ "", tijmath::Position{ tijmath::Vector3{ 10., 20., 30. } },
                                         tijmath::Rotation::Identity };

  {
    // From occupied to emtpy, should work just fine
    auto src = ManagedLocus::CreatePartLocus(src_parent, src_pose, part_id_, true);
    auto dst = ManagedLocus::CreateEmptyLocus(dst_parent, dst_pose);

    ASSERT_TRUE(is_occupied_space(src, part_id_, true));
    ASSERT_TRUE(is_the_same_pose(src_pose, src.pose(), tolerance_));
    ASSERT_EQ(src_parent, src.parentName());
    ASSERT_TRUE(is_empty_space(dst));
    ASSERT_TRUE(is_the_same_pose(dst_pose, dst.pose(), tolerance_));
    ASSERT_EQ(dst_parent, dst.parentName());

    ManagedLocus::TransferPartFromHereToThere(src, dst);

    ASSERT_TRUE(is_empty_space(src));
    ASSERT_TRUE(is_the_same_pose(src_pose, src.pose(), tolerance_));
    ASSERT_EQ(src_parent, src.parentName());
    ASSERT_TRUE(is_occupied_space(dst, part_id_, true));
    ASSERT_TRUE(is_the_same_pose(dst_pose, dst.pose(), tolerance_));
    ASSERT_EQ(dst_parent, dst.parentName());
  }

  {
    // From empty to occupied, should throw
    auto src = ManagedLocus::CreateEmptyLocus(src_parent, src_pose);
    auto dst = ManagedLocus::CreatePartLocus(dst_parent, dst_pose, part_id_, true);
    ASSERT_TRUE(is_empty_space(src));
    ASSERT_TRUE(is_the_same_pose(src_pose, src.pose(), tolerance_));
    ASSERT_EQ(src_parent, src.parentName());
    ASSERT_TRUE(is_occupied_space(dst, part_id_, true));
    ASSERT_TRUE(is_the_same_pose(dst_pose, dst.pose(), tolerance_));
    ASSERT_EQ(dst_parent, dst.parentName());

    ASSERT_THROW({ ManagedLocus::TransferPartFromHereToThere(src, dst); }, std::logic_error);
  }

  {
    // From occupied to occupied, should throw
    auto src = ManagedLocus::CreatePartLocus(src_parent, src_pose, part_id_, true);
    auto dst = ManagedLocus::CreatePartLocus(dst_parent, dst_pose, part_id_, true);
    ASSERT_TRUE(is_occupied_space(src, part_id_, true));
    ASSERT_TRUE(is_the_same_pose(src_pose, src.pose(), tolerance_));
    ASSERT_EQ(src_parent, src.parentName());
    ASSERT_TRUE(is_occupied_space(dst, part_id_, true));
    ASSERT_TRUE(is_the_same_pose(dst_pose, dst.pose(), tolerance_));
    ASSERT_EQ(dst_parent, dst.parentName());

    ASSERT_THROW({ ManagedLocus::TransferPartFromHereToThere(src, dst); }, std::logic_error);
  }
}

}  // namespace

}  // namespace test

}  // namespace utils

}  // namespace tijcore
