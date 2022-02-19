/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// gtest
#include "gtest/gtest.h"

// tijcore
#include "mocks/ModelContainerMock.hpp"

namespace tijcore
{
namespace test
{
namespace
{
using ::testing::Test;

class ModelContainerInterfaceTests : public Test
{
protected:
  const double position_tolerance_{ 1e-6 };
  const double angular_tolerance_{ 1e-6 };
};

TEST_F(ModelContainerInterfaceTests, ConstructionTest)
{
  const tijmath::RelativePose3 uut_pose{ "frame_id", tijmath::Position::fromVector(1, 2, 3),
                                         tijmath::Rotation::fromQuaternion(0, 0, 1, 0) };

  const tijmath::Vector3 lbr_corner{ -2, -3, -1 };
  const tijmath::Vector3 ufl_corner{ 3, 4, 2 };
  const CuboidVolume uut_container_volume{ lbr_corner, ufl_corner };

  const ModelContainerMock uut{
    "uut_name", "container_reference_frame_id", "surface_reference_frame_id",
    uut_pose,   uut_container_volume,           "exclusion_volume_id"
  };

  ASSERT_EQ("uut_name", uut.name());
  ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(uut_pose, uut.pose(), position_tolerance_,
                                                        angular_tolerance_));

  ASSERT_TRUE(tijmath::Position::samePosition(tijmath::Position(lbr_corner),
                                              uut.containerVolume().lowerRightBackCorner(),
                                              position_tolerance_));
  ASSERT_TRUE(tijmath::Position::samePosition(tijmath::Position(ufl_corner),
                                              uut.containerVolume().upperLeftFrontCorner(),
                                              position_tolerance_));
}

}  // namespace

}  // namespace test

}  // namespace tijcore
