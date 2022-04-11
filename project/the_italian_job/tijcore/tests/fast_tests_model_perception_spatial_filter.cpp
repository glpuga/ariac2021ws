/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// standard library
#include <initializer_list>
#include <memory>
#include <utility>
#include <vector>

// gtest
#include "gmock/gmock.h"
#include "gtest/gtest.h"

// tijcore
#include <tijcore/coremodels/ModelPerceptionSpatialFilter.hpp>
#include <tijcore/datatypes/QualifiedPartInfo.hpp>
#include <tijcore/utils/StaticFrameTransformer.hpp>
#include <tijlogger/logger.hpp>

// mocks
#include "mocks/BlindVolumeTrackerMock.hpp"
#include "mocks/ModelPerceptionMock.hpp"

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

struct ModelPerceptionSpatialFilterTests : public Test
{
  using VectorOfModels = std::vector<ObservedItem>;

  std::shared_ptr<StaticFrameTransformer> static_frame_transformer_;
  std::shared_ptr<Toolbox> toolbox_;

  std::unique_ptr<BlindVolumeTrackerMock> robot_1_tracker_;
  std::unique_ptr<BlindVolumeTrackerMock> robot_2_tracker_;

  ModelPerceptionSpatialFilterTests()
  {
    static_frame_transformer_ = std::make_shared<StaticFrameTransformer>(
        std::initializer_list<StaticFrameTransformer::TransformTreeLink>{
            { "camera", tijmath::RelativePose3{ "world", tijmath::Position::fromVector(0, 0, 1),
                                                tijmath::Rotation::Identity } },
            {
                "robot_1",
                tijmath::RelativePose3{ "world", tijmath::Position::fromVector(1, 1, 2),
                                        tijmath::Rotation{ tijmath::Rotation::Identity } },
            },
            {
                "robot_2",
                tijmath::RelativePose3{ "world", tijmath::Position::fromVector(0, 1, 2),
                                        tijmath::Rotation{ tijmath::Rotation::Identity } },
            },
        });

    Toolbox::Contents contents;
    contents.frame_transformer_instance = static_frame_transformer_;
    toolbox_ = std::make_shared<Toolbox>(std::move(contents));

    tijmath::RelativePose3 robot_1_pose{ "robot_1", tijmath::Position::fromVector(0, 0, 0),
                                         tijmath::Rotation::Identity };
    const double robot_1_radius = 0.1;
    robot_1_tracker_ = std::make_unique<BlindVolumeTrackerMock>();
    EXPECT_CALL(*robot_1_tracker_, pose()).WillRepeatedly(Return(robot_1_pose));
    EXPECT_CALL(*robot_1_tracker_, radius()).WillRepeatedly(Return(robot_1_radius));

    tijmath::RelativePose3 robot_2_pose{ "robot_2", tijmath::Position::fromVector(0, 0, 0),
                                         tijmath::Rotation::Identity };
    const double robot_2_radius = 0.2;
    robot_2_tracker_ = std::make_unique<BlindVolumeTrackerMock>();
    EXPECT_CALL(*robot_2_tracker_, pose()).WillRepeatedly(Return(robot_2_pose));
    EXPECT_CALL(*robot_2_tracker_, radius()).WillRepeatedly(Return(robot_2_radius));
  }
};

TEST_F(ModelPerceptionSpatialFilterTests, EmptyItemsInVector)
{
  InSequence seq;

  auto mock_child = std::make_unique<ModelPerceptionMock>();

  tijmath::RelativePose3 pose_1_blind{ "camera", tijmath::Position::fromVector(1, 1, 1),
                                       tijmath::Rotation::Identity };
  tijmath::RelativePose3 pose_2_blind{ "camera", tijmath::Position::fromVector(0.91, 1, 1),
                                       tijmath::Rotation::Identity };
  tijmath::RelativePose3 pose_3_visible{ "camera", tijmath::Position::fromVector(0.89, 1, 1),
                                         tijmath::Rotation::Identity };
  tijmath::RelativePose3 pose_4_blind{ "camera", tijmath::Position::fromVector(0, 1.00, 1),
                                       tijmath::Rotation::Identity };
  tijmath::RelativePose3 pose_5_blind{ "camera", tijmath::Position::fromVector(0, 1.19, 1),
                                       tijmath::Rotation::Identity };
  tijmath::RelativePose3 pose_6_visible{ "camera", tijmath::Position::fromVector(0, 1.21, 1),
                                         tijmath::Rotation::Identity };

  {
    VectorOfModels mocked_output = {
      { QualifiedPartInfo{ PartId("assembly_pump_red") }, pose_1_blind },
    };
    EXPECT_CALL(*mock_child, getObservedModels()).WillOnce(Return(mocked_output));
  }
  {
    VectorOfModels mocked_output = {
      { QualifiedPartInfo{ PartId("assembly_pump_red") }, pose_2_blind },
    };
    EXPECT_CALL(*mock_child, getObservedModels()).WillOnce(Return(mocked_output));
  }
  {
    VectorOfModels mocked_output = {
      { QualifiedPartInfo{ PartId("assembly_pump_red") }, pose_3_visible },
    };
    EXPECT_CALL(*mock_child, getObservedModels()).WillOnce(Return(mocked_output));
  }
  {
    VectorOfModels mocked_output = {
      { QualifiedPartInfo{ PartId("assembly_pump_red") }, pose_4_blind },
    };
    EXPECT_CALL(*mock_child, getObservedModels()).WillOnce(Return(mocked_output));
  }
  {
    VectorOfModels mocked_output = {
      { QualifiedPartInfo{ PartId("assembly_pump_red") }, pose_5_blind },
    };
    EXPECT_CALL(*mock_child, getObservedModels()).WillOnce(Return(mocked_output));
  }
  {
    VectorOfModels mocked_output = {
      { QualifiedPartInfo{ PartId("assembly_pump_red") }, pose_6_visible },
    };
    EXPECT_CALL(*mock_child, getObservedModels()).WillOnce(Return(mocked_output));
  }

  {
    VectorOfModels mocked_output = {
      { QualifiedPartInfo{ PartId("assembly_pump_red") }, pose_1_blind },
      { QualifiedPartInfo{ PartId("assembly_pump_red") }, pose_2_blind },
      { QualifiedPartInfo{ PartId("assembly_pump_red") }, pose_3_visible },
      { QualifiedPartInfo{ PartId("assembly_pump_red") }, pose_4_blind },
      { QualifiedPartInfo{ PartId("assembly_pump_red") }, pose_5_blind },
      { QualifiedPartInfo{ PartId("assembly_pump_red") }, pose_6_visible },
    };
    EXPECT_CALL(*mock_child, getObservedModels()).WillOnce(Return(mocked_output));
  }

  const auto uut = std::make_unique<ModelPerceptionSpatialFilter>(toolbox_, std::move(mock_child));

  uut->addBlindVolumeTracker(std::move(robot_1_tracker_));
  uut->addBlindVolumeTracker(std::move(robot_2_tracker_));

  ASSERT_EQ(0u, uut->getObservedModels().size());
  ASSERT_EQ(0u, uut->getObservedModels().size());
  ASSERT_EQ(1u, uut->getObservedModels().size());
  ASSERT_EQ(0u, uut->getObservedModels().size());
  ASSERT_EQ(0u, uut->getObservedModels().size());
  ASSERT_EQ(1u, uut->getObservedModels().size());
  ASSERT_EQ(2u, uut->getObservedModels().size());
}

}  // namespace

}  // namespace test

}  // namespace utils

}  // namespace tijcore
