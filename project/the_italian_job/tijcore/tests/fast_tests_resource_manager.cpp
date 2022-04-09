/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// standard library
#include <functional>
#include <iostream>
#include <list>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

// gtest
#include "gtest/gtest.h"

// tijcore
#include <tijcore/coremodels/Toolbox.hpp>
#include <tijcore/datatypes/QualifiedPartInfo.hpp>
#include <tijcore/resources/ResourceManager.hpp>
#include <tijcore/utils/StaticFrameTransformer.hpp>

#include "mocks/ModelContainerMock.hpp"
#include "mocks/PickAndPlaceRobotMock.hpp"
#include "utils/ActionQueue.hpp"

namespace tijcore
{
namespace test
{
namespace
{
using ::testing::InSequence;
using ::testing::Return;
using ::testing::Test;

class ResourceManagerTests : public Test
{
public:
  const double position_tolerance_{ 1e-3 };
  const double rotation_tolerance_{ 1e-3 };

  const double empty_radius_{ 0.2 };

  const std::chrono::seconds access_timeout_{ 1 };

  const CuboidVolume table_container_volume{ tijmath::Vector3{ 0, 0, -0.1 },
                                             tijmath::Vector3{ 0.9, 0.9, 0.1 } };

  const tijmath::Pose3 table_rel_pose_11{ tijmath::Position::fromVector(0.22, 0.22, 0),
                                          tijmath::Rotation::fromQuaternion(0, 0, 0, 1) };
  const tijmath::Pose3 table_rel_pose_12{ tijmath::Position::fromVector(0.68, 0.22, 0),
                                          tijmath::Rotation::fromQuaternion(0, 0, 0, 1) };
  const tijmath::Pose3 table_rel_pose_21{ tijmath::Position::fromVector(0.22, 0.68, 0),
                                          tijmath::Rotation::fromQuaternion(0, 0, 0, 1) };
  const tijmath::Pose3 table_rel_pose_22{ tijmath::Position::fromVector(0.68, 0.68, 0),
                                          tijmath::Rotation::fromQuaternion(0, 0, 0, 1) };

  const PartId part_rpump_{ PartTypeId::pump, PartColorId::red };
  const PartId part_gpump_{ PartTypeId::pump, PartColorId::green };
  const PartId part_bpump_{ PartTypeId::pump, PartColorId::blue };
  const PartId part_rbatt_{ PartTypeId::battery, PartColorId::red };
  const PartId part_gbatt_{ PartTypeId::battery, PartColorId::green };
  const PartId part_bbatt_{ PartTypeId::battery, PartColorId::blue };

  // table 1
  const std::string table_1_name_{ "table1_name" };
  const std::string table_1_frame_id_{ "table1_frame" };
  const tijmath::RelativePose3 table_1_pose_{ "world", tijmath::Position::fromVector(0, 0, 1), {} };
  ModelContainerMock::Ptr table_1_container_mock_;
  CuboidVolume table_1_container_volume_{ table_container_volume };
  const std::string table_1_exclusion_volume_{ "table1_exclusion_zone" };

  // table 2
  const std::string table_2_name_{ "table2_name" };
  const std::string table_2_frame_id_{ "table2_frame" };
  const tijmath::RelativePose3 table_2_pose_{ "world", tijmath::Position::fromVector(1, 0, 1), {} };
  ModelContainerMock::Ptr table_2_container_mock_;
  CuboidVolume table_2_container_volume_{ table_container_volume };
  const std::string table_2_exclusion_volume_{ "table2_exclusion_zone" };

  // table 3
  const std::string table_3_name_{ "table3_name" };
  const std::string table_3_frame_id_{ "table3_frame" };
  const tijmath::RelativePose3 table_3_pose_{ "world", tijmath::Position::fromVector(2, 0, 1), {} };
  ModelContainerMock::Ptr table_3_container_mock_;
  CuboidVolume table_3_container_volume_{ table_container_volume };
  const std::string table_3_exclusion_volume_{ "table3_exclusion_zone" };

  // robots
  PickAndPlaceRobotMock::Ptr kitting_robot_mock_;
  PickAndPlaceRobotMock::Ptr assembly_robot_mock_;

  // toolbox stuff
  std::shared_ptr<StaticFrameTransformer> static_frame_transformer_;
  std::shared_ptr<Toolbox> toolbox_;

  std::unique_ptr<ResourceManagerInterface> uut_;

  utils::ActionQueue action_queue_;

  void SetUp() override
  {
    static_frame_transformer_ = std::make_shared<StaticFrameTransformer>(
        std::initializer_list<StaticFrameTransformer::TransformTreeLink>{
            { table_1_frame_id_, table_1_pose_ },
            { table_2_frame_id_, table_2_pose_ },
            { table_3_frame_id_, table_3_pose_ },
        });

    table_1_container_mock_ = std::make_unique<ModelContainerMock>(
        table_1_name_, table_1_frame_id_, table_1_frame_id_, table_1_pose_,
        table_1_container_volume_, table_1_exclusion_volume_);

    table_2_container_mock_ = std::make_unique<ModelContainerMock>(
        table_2_name_, table_2_frame_id_, table_2_frame_id_, table_2_pose_,
        table_2_container_volume_, table_2_exclusion_volume_);

    table_3_container_mock_ = std::make_unique<ModelContainerMock>(
        table_3_name_, table_3_frame_id_, table_3_frame_id_, table_3_pose_,
        table_3_container_volume_, table_3_exclusion_volume_);

    kitting_robot_mock_ = std::make_unique<PickAndPlaceRobotMock>();
    assembly_robot_mock_ = std::make_unique<PickAndPlaceRobotMock>();

    Toolbox::Contents contents;
    contents.frame_transformer_instance = static_frame_transformer_;

    toolbox_ = std::make_shared<Toolbox>(std::move(contents));
  }

  void buildUnitUnderTest()
  {
    std::vector<ModelContainerInterface::Ptr> containers_;
    containers_.emplace_back(std::move(table_1_container_mock_));
    containers_.emplace_back(std::move(table_2_container_mock_));
    containers_.emplace_back(std::move(table_3_container_mock_));

    std::vector<PickAndPlaceRobotInterface::Ptr> robots_;
    robots_.push_back(std::move(kitting_robot_mock_));
    robots_.push_back(std::move(assembly_robot_mock_));

    uut_ = std::make_unique<ResourceManager>(toolbox_, std::move(containers_), std::move(robots_));
  }
};

TEST_F(ResourceManagerTests, ConstructionDestruction)
{
  buildUnitUnderTest();
  uut_ = nullptr;
}

TEST_F(ResourceManagerTests, StartuptState)
{
  EXPECT_CALL(*table_1_container_mock_, enabled()).WillRepeatedly(Return(true));
  EXPECT_CALL(*table_2_container_mock_, enabled()).WillRepeatedly(Return(true));
  EXPECT_CALL(*table_3_container_mock_, enabled()).WillRepeatedly(Return(true));
  buildUnitUnderTest();

  auto empty_loci = uut_->findEmptyLoci(empty_radius_);
  ASSERT_EQ(3u, empty_loci.size());

  auto rpump = uut_->findManagedLociByPartId(part_rpump_);
  auto gpump = uut_->findManagedLociByPartId(part_gpump_);
  auto bpump = uut_->findManagedLociByPartId(part_bpump_);
  ASSERT_EQ(0u, rpump.size());
  ASSERT_EQ(0u, gpump.size());
  ASSERT_EQ(0u, bpump.size());
}

// disabled because we now use an empty list as a proxy for a blacked
// out list of items.
TEST_F(ResourceManagerTests, DISABLED_SensorDataWithNoTablesInUse)
{
  EXPECT_CALL(*table_1_container_mock_, enabled()).WillRepeatedly(Return(true));
  EXPECT_CALL(*table_2_container_mock_, enabled()).WillRepeatedly(Return(true));
  EXPECT_CALL(*table_3_container_mock_, enabled()).WillRepeatedly(Return(true));
  buildUnitUnderTest();

  {
    auto rpump = uut_->findManagedLociByPartId(part_rpump_);
    ASSERT_EQ(0u, rpump.size());
  }

  std::vector<ObservedItem> observed_models_ = {
    { QualifiedPartInfo{ part_rpump_ },
      tijmath::RelativePose3(table_1_frame_id_, table_rel_pose_11) },
    { QualifiedPartInfo{ part_rpump_ },
      tijmath::RelativePose3(table_2_frame_id_, table_rel_pose_11) },
    { QualifiedPartInfo{ part_rpump_ },
      tijmath::RelativePose3(table_2_frame_id_, table_rel_pose_22) },
    { QualifiedPartInfo{ part_rpump_ },
      tijmath::RelativePose3(table_3_frame_id_, table_rel_pose_22) },
  };
  uut_->updateSensorData(observed_models_);

  {
    auto rpump = uut_->findManagedLociByPartId(part_rpump_);
    ASSERT_EQ(4u, rpump.size());
  }

  uut_->updateSensorData({});

  {
    auto rpump = uut_->findManagedLociByPartId(part_rpump_);
    ASSERT_EQ(0u, rpump.size());
  }

  observed_models_ = {
    { QualifiedPartInfo{ part_bpump_ },
      tijmath::RelativePose3(table_1_frame_id_, table_rel_pose_11) },
    { QualifiedPartInfo{ part_bpump_ },
      tijmath::RelativePose3(table_2_frame_id_, table_rel_pose_22) },
    { QualifiedPartInfo{ part_gpump_ },
      tijmath::RelativePose3(table_3_frame_id_, table_rel_pose_11) },
    { QualifiedPartInfo{ part_gpump_ },
      tijmath::RelativePose3(table_3_frame_id_, table_rel_pose_12) },
    { QualifiedPartInfo{ part_gpump_ },
      tijmath::RelativePose3(table_3_frame_id_, table_rel_pose_22) },
  };
  uut_->updateSensorData(observed_models_);

  {
    auto rpump = uut_->findManagedLociByPartId(part_rpump_);
    auto gpump = uut_->findManagedLociByPartId(part_gpump_);
    auto bpump = uut_->findManagedLociByPartId(part_bpump_);
    ASSERT_EQ(0u, rpump.size());
    ASSERT_EQ(3u, gpump.size());
    ASSERT_EQ(2u, bpump.size());
  }
}

TEST_F(ResourceManagerTests, SensorDataUpdateKnownData)
{
  EXPECT_CALL(*table_1_container_mock_, enabled()).WillRepeatedly(Return(true));
  EXPECT_CALL(*table_2_container_mock_, enabled()).WillRepeatedly(Return(true));
  EXPECT_CALL(*table_3_container_mock_, enabled()).WillRepeatedly(Return(true));
  buildUnitUnderTest();

  const PartId part1{ part_rpump_ };
  const PartId part2{ part_gpump_ };
  const PartId part3{ part_rbatt_ };
  const PartId part4{ part_gbatt_ };

  const tijmath::RelativePose3 pose1(table_1_frame_id_, table_rel_pose_11);
  const tijmath::RelativePose3 pose2(table_1_frame_id_, table_rel_pose_12);
  const tijmath::RelativePose3 pose3(table_1_frame_id_, table_rel_pose_21);
  const tijmath::RelativePose3 pose4(table_1_frame_id_, table_rel_pose_22);

  {
    std::vector<ObservedItem> observed_models_ = {
      { QualifiedPartInfo{ part1 }, pose1 },
      { QualifiedPartInfo{ part2 }, pose2 },
      { QualifiedPartInfo{ part3 }, pose3 },
      { QualifiedPartInfo{ part4 }, pose4 },
    };
    uut_->updateSensorData(observed_models_);

    auto part1_handlers = uut_->findManagedLociByPartId(part1);
    auto part2_handlers = uut_->findManagedLociByPartId(part2);
    auto part3_handlers = uut_->findManagedLociByPartId(part3);
    auto part4_handlers = uut_->findManagedLociByPartId(part4);
    ASSERT_EQ(1u, part1_handlers.size());
    ASSERT_EQ(1u, part2_handlers.size());
    ASSERT_EQ(1u, part3_handlers.size());
    ASSERT_EQ(1u, part4_handlers.size());
    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(
        pose1, part1_handlers[0].resource()->pose(), position_tolerance_, rotation_tolerance_));
    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(
        pose2, part2_handlers[0].resource()->pose(), position_tolerance_, rotation_tolerance_));
    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(
        pose3, part3_handlers[0].resource()->pose(), position_tolerance_, rotation_tolerance_));
    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(
        pose4, part4_handlers[0].resource()->pose(), position_tolerance_, rotation_tolerance_));
  }

  {
    std::vector<ObservedItem> observed_models_ = {
      { QualifiedPartInfo{ part2 }, pose1 },
      { QualifiedPartInfo{ part3 }, pose2 },
      { QualifiedPartInfo{ part4 }, pose3 },
      { QualifiedPartInfo{ part1 }, pose4 },
    };
    uut_->updateSensorData(observed_models_);

    auto part1_handlers = uut_->findManagedLociByPartId(part1);
    auto part2_handlers = uut_->findManagedLociByPartId(part2);
    auto part3_handlers = uut_->findManagedLociByPartId(part3);
    auto part4_handlers = uut_->findManagedLociByPartId(part4);
    ASSERT_EQ(1u, part1_handlers.size());
    ASSERT_EQ(1u, part2_handlers.size());
    ASSERT_EQ(1u, part3_handlers.size());
    ASSERT_EQ(1u, part4_handlers.size());
    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(
        pose4, part1_handlers[0].resource()->pose(), position_tolerance_, rotation_tolerance_));
    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(
        pose1, part2_handlers[0].resource()->pose(), position_tolerance_, rotation_tolerance_));
    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(
        pose2, part3_handlers[0].resource()->pose(), position_tolerance_, rotation_tolerance_));
    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(
        pose3, part4_handlers[0].resource()->pose(), position_tolerance_, rotation_tolerance_));
  }

  {
    std::vector<ObservedItem> observed_models_ = {
      { QualifiedPartInfo{ part3 }, pose1 },
      { QualifiedPartInfo{ part4 }, pose2 },
      { QualifiedPartInfo{ part1 }, pose3 },
      { QualifiedPartInfo{ part2 }, pose4 },
    };
    uut_->updateSensorData(observed_models_);

    auto part1_handlers = uut_->findManagedLociByPartId(part1);
    auto part2_handlers = uut_->findManagedLociByPartId(part2);
    auto part3_handlers = uut_->findManagedLociByPartId(part3);
    auto part4_handlers = uut_->findManagedLociByPartId(part4);
    ASSERT_EQ(1u, part1_handlers.size());
    ASSERT_EQ(1u, part2_handlers.size());
    ASSERT_EQ(1u, part3_handlers.size());
    ASSERT_EQ(1u, part4_handlers.size());
    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(
        pose3, part1_handlers[0].resource()->pose(), position_tolerance_, rotation_tolerance_));
    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(
        pose4, part2_handlers[0].resource()->pose(), position_tolerance_, rotation_tolerance_));
    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(
        pose1, part3_handlers[0].resource()->pose(), position_tolerance_, rotation_tolerance_));
    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(
        pose2, part4_handlers[0].resource()->pose(), position_tolerance_, rotation_tolerance_));
  }
}

TEST_F(ResourceManagerTests, SensorDataMissingKnownModelGetPurged)
{
  EXPECT_CALL(*table_1_container_mock_, enabled()).WillRepeatedly(Return(true));
  EXPECT_CALL(*table_2_container_mock_, enabled()).WillRepeatedly(Return(true));
  EXPECT_CALL(*table_3_container_mock_, enabled()).WillRepeatedly(Return(true));
  buildUnitUnderTest();

  const PartId part1{ part_rpump_ };
  const PartId part2{ part_gpump_ };
  const PartId part3{ part_rbatt_ };
  const PartId part4{ part_gbatt_ };

  const tijmath::RelativePose3 pose1(table_1_frame_id_, table_rel_pose_11);
  const tijmath::RelativePose3 pose2(table_1_frame_id_, table_rel_pose_12);
  const tijmath::RelativePose3 pose3(table_1_frame_id_, table_rel_pose_21);
  const tijmath::RelativePose3 pose4(table_1_frame_id_, table_rel_pose_22);

  {
    std::vector<ObservedItem> observed_models_ = {
      { QualifiedPartInfo{ part1 }, pose1 },
      { QualifiedPartInfo{ part2 }, pose2 },
      { QualifiedPartInfo{ part3 }, pose3 },
      { QualifiedPartInfo{ part4 }, pose4 },
    };
    uut_->updateSensorData(observed_models_);

    auto part1_handlers = uut_->findManagedLociByPartId(part1);
    auto part2_handlers = uut_->findManagedLociByPartId(part2);
    auto part3_handlers = uut_->findManagedLociByPartId(part3);
    auto part4_handlers = uut_->findManagedLociByPartId(part4);
    ASSERT_EQ(1u, part1_handlers.size());
    ASSERT_EQ(1u, part2_handlers.size());
    ASSERT_EQ(1u, part3_handlers.size());
    ASSERT_EQ(1u, part4_handlers.size());
    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(
        pose1, part1_handlers[0].resource()->pose(), position_tolerance_, rotation_tolerance_));
    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(
        pose2, part2_handlers[0].resource()->pose(), position_tolerance_, rotation_tolerance_));
    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(
        pose3, part3_handlers[0].resource()->pose(), position_tolerance_, rotation_tolerance_));
    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(
        pose4, part4_handlers[0].resource()->pose(), position_tolerance_, rotation_tolerance_));
  }

  {
    std::vector<ObservedItem> observed_models_ = {
      { QualifiedPartInfo{ part1 }, pose1 },
      { QualifiedPartInfo{ part2 }, pose2 },
      { QualifiedPartInfo{ part4 }, pose4 },
    };
    uut_->updateSensorData(observed_models_);

    auto part1_handlers = uut_->findManagedLociByPartId(part1);
    auto part2_handlers = uut_->findManagedLociByPartId(part2);
    auto part3_handlers = uut_->findManagedLociByPartId(part3);
    auto part4_handlers = uut_->findManagedLociByPartId(part4);
    ASSERT_EQ(1u, part1_handlers.size());
    ASSERT_EQ(1u, part2_handlers.size());
    ASSERT_EQ(0u, part3_handlers.size());
    ASSERT_EQ(1u, part4_handlers.size());
    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(
        pose1, part1_handlers[0].resource()->pose(), position_tolerance_, rotation_tolerance_));
    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(
        pose2, part2_handlers[0].resource()->pose(), position_tolerance_, rotation_tolerance_));
    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(
        pose4, part4_handlers[0].resource()->pose(), position_tolerance_, rotation_tolerance_));
  }
}

TEST_F(ResourceManagerTests, DISABLED_CreateTargetReturnsKnownLoci)
{
  EXPECT_CALL(*table_1_container_mock_, enabled()).WillRepeatedly(Return(true));
  EXPECT_CALL(*table_2_container_mock_, enabled()).WillRepeatedly(Return(true));
  EXPECT_CALL(*table_3_container_mock_, enabled()).WillRepeatedly(Return(true));
  buildUnitUnderTest();

  const PartId part1{ part_rpump_ };
  const PartId part2{ part_gpump_ };
  const PartId part3{ part_rbatt_ };
  const PartId part4{ part_gbatt_ };

  const tijmath::RelativePose3 pose1(table_1_frame_id_, table_rel_pose_11);
  const tijmath::RelativePose3 pose2(table_1_frame_id_, table_rel_pose_12);
  const tijmath::RelativePose3 pose3(table_1_frame_id_, table_rel_pose_21);
  const tijmath::RelativePose3 pose4(table_1_frame_id_, table_rel_pose_22);

  // Test we can allocate a target in empty space
  {
    auto handle1 = uut_->getManagedLocusHandleForPose(pose1);
    auto handle2 = uut_->getManagedLocusHandleForPose(pose2);
    auto handle3 = uut_->getManagedLocusHandleForPose(pose3);
    auto handle4 = uut_->getManagedLocusHandleForPose(pose4);

    ASSERT_NE(std::nullopt, handle1);
    ASSERT_NE(std::nullopt, handle2);
    ASSERT_NE(std::nullopt, handle3);
    ASSERT_NE(std::nullopt, handle4);

    ASSERT_TRUE(handle1->resource()->isEmpty());
    ASSERT_TRUE(handle2->resource()->isEmpty());
    ASSERT_TRUE(handle3->resource()->isEmpty());
    ASSERT_TRUE(handle4->resource()->isEmpty());
  }

  // test that for an existing model locus we get the handle
  // with the information for that locus if it is not
  // allocated
  {
    std::vector<ObservedItem> observed_models_ = {
      { QualifiedPartInfo{ part1 }, pose1 },
      { QualifiedPartInfo{ part2 }, pose2 },
      { QualifiedPartInfo{ part3 }, pose3 },
      { QualifiedPartInfo{ part4 }, pose4 },
    };
    uut_->updateSensorData(observed_models_);

    auto handle1 = uut_->getManagedLocusHandleForPose(pose1);
    auto handle2 = uut_->getManagedLocusHandleForPose(pose2);
    auto handle3 = uut_->getManagedLocusHandleForPose(pose3);
    auto handle4 = uut_->getManagedLocusHandleForPose(pose4);

    ASSERT_NE(std::nullopt, handle1);
    ASSERT_NE(std::nullopt, handle2);
    ASSERT_NE(std::nullopt, handle3);
    ASSERT_NE(std::nullopt, handle4);

    ASSERT_TRUE(handle1->resource()->isModel());
    ASSERT_TRUE(handle2->resource()->isModel());
    ASSERT_TRUE(handle3->resource()->isModel());
    ASSERT_TRUE(handle4->resource()->isModel());

    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(
        pose1, handle1->resource()->pose(), position_tolerance_, rotation_tolerance_));
    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(
        pose2, handle2->resource()->pose(), position_tolerance_, rotation_tolerance_));
    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(
        pose3, handle3->resource()->pose(), position_tolerance_, rotation_tolerance_));
    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(
        pose4, handle4->resource()->pose(), position_tolerance_, rotation_tolerance_));
  }

  // that that if the pose match that of a known model locus, but the
  // handle is allocated, we get nullopt
  {
    std::vector<ObservedItem> observed_models_ = {
      { QualifiedPartInfo{ part1 }, pose1 },
      { QualifiedPartInfo{ part2 }, pose2 },
      { QualifiedPartInfo{ part3 }, pose3 },
      { QualifiedPartInfo{ part4 }, pose4 },
    };
    uut_->updateSensorData(observed_models_);

    auto allocated_part1 = uut_->findManagedLociByPartId(part1);
    auto allocated_part3 = uut_->findManagedLociByPartId(part3);
    ASSERT_EQ(1u, allocated_part1.size());
    ASSERT_EQ(1u, allocated_part3.size());

    auto handle1 = uut_->getManagedLocusHandleForPose(pose1);
    auto handle2 = uut_->getManagedLocusHandleForPose(pose2);
    auto handle3 = uut_->getManagedLocusHandleForPose(pose3);
    auto handle4 = uut_->getManagedLocusHandleForPose(pose4);

    ASSERT_EQ(std::nullopt, handle1);  // is allocated => nullopt
    ASSERT_NE(std::nullopt, handle2);
    ASSERT_EQ(std::nullopt, handle3);  // is allocated => nullopt
    ASSERT_NE(std::nullopt, handle4);

    ASSERT_TRUE(handle2->resource()->isModel());
    ASSERT_TRUE(handle4->resource()->isModel());

    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(
        pose2, handle2->resource()->pose(), position_tolerance_, rotation_tolerance_));
    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(
        pose4, handle4->resource()->pose(), position_tolerance_, rotation_tolerance_));
  }
}

TEST_F(ResourceManagerTests, GetRobotChoosesTheBestMinimumFit)
{
  InSequence aux;

  EXPECT_CALL(*kitting_robot_mock_, enabled()).WillOnce(Return(false));
  EXPECT_CALL(*assembly_robot_mock_, enabled()).WillOnce(Return(false));

  action_queue_.queueTestActionQueue([this] {
    const std::set<WorkRegionId> requested_set{
      WorkRegionId::assembly,
      WorkRegionId::kitting_near_bins,
      WorkRegionId::conveyor_belt,
    };
    auto pap_robot_handle = uut_->getPickAndPlaceRobotHandle(requested_set);
    EXPECT_EQ(std::nullopt, pap_robot_handle);
  });

  EXPECT_CALL(*kitting_robot_mock_, enabled()).WillOnce(Return(true));
  EXPECT_CALL(*kitting_robot_mock_, supportedRegions())
      .WillOnce(Return(std::set<WorkRegionId>{
          WorkRegionId::assembly,
          WorkRegionId::kitting_near_bins,
      }));
  EXPECT_CALL(*assembly_robot_mock_, enabled()).WillOnce(Return(false));

  action_queue_.queueTestActionQueue([this] {
    const std::set<WorkRegionId> requested_set{
      WorkRegionId::assembly,
      WorkRegionId::kitting_near_bins,
      WorkRegionId::conveyor_belt,
    };
    auto pap_robot_handle = uut_->getPickAndPlaceRobotHandle(requested_set);
    EXPECT_EQ(std::nullopt, pap_robot_handle);
  });

  EXPECT_CALL(*kitting_robot_mock_, enabled()).WillOnce(Return(false));
  EXPECT_CALL(*assembly_robot_mock_, enabled()).WillOnce(Return(true));
  EXPECT_CALL(*assembly_robot_mock_, supportedRegions())
      .WillOnce(Return(std::set<WorkRegionId>{
          WorkRegionId::assembly,
          WorkRegionId::kitting_near_bins,
      }));

  action_queue_.queueTestActionQueue([this] {
    const std::set<WorkRegionId> requested_set{
      WorkRegionId::assembly,
      WorkRegionId::kitting_near_bins,
      WorkRegionId::conveyor_belt,
    };
    auto pap_robot_handle = uut_->getPickAndPlaceRobotHandle(requested_set);
    EXPECT_EQ(std::nullopt, pap_robot_handle);
  });

  EXPECT_CALL(*kitting_robot_mock_, enabled()).WillOnce(Return(true));
  EXPECT_CALL(*kitting_robot_mock_, supportedRegions())
      .WillOnce(Return(std::set<WorkRegionId>{
          WorkRegionId::assembly,
          WorkRegionId::kitting_near_bins,
      }));
  EXPECT_CALL(*assembly_robot_mock_, enabled()).WillOnce(Return(true));
  EXPECT_CALL(*assembly_robot_mock_, supportedRegions())
      .WillOnce(Return(std::set<WorkRegionId>{
          WorkRegionId::assembly,
          WorkRegionId::kitting_near_bins,
      }));

  action_queue_.queueTestActionQueue([this] {
    const std::set<WorkRegionId> requested_set{
      WorkRegionId::assembly,
      WorkRegionId::kitting_near_bins,
      WorkRegionId::conveyor_belt,
    };
    auto pap_robot_handle = uut_->getPickAndPlaceRobotHandle(requested_set);
    EXPECT_EQ(std::nullopt, pap_robot_handle);
  });

  EXPECT_CALL(*kitting_robot_mock_, enabled()).WillOnce(Return(true));
  EXPECT_CALL(*kitting_robot_mock_, supportedRegions())
      .WillOnce(Return(std::set<WorkRegionId>{
          WorkRegionId::assembly,
          WorkRegionId::kitting_near_bins,
          WorkRegionId::conveyor_belt,
      }));
  EXPECT_CALL(*assembly_robot_mock_, enabled()).WillOnce(Return(true));
  EXPECT_CALL(*assembly_robot_mock_, supportedRegions())
      .WillOnce(Return(std::set<WorkRegionId>{
          WorkRegionId::assembly,
          WorkRegionId::kitting_near_bins,
      }));

  // This set is only to check that the correct robot handle is returned, by
  // checking that the correct mock is accessed; the value returned is not
  // important, only that it does get called.
  EXPECT_CALL(*kitting_robot_mock_, supportedRegions()).WillOnce(Return(std::set<WorkRegionId>{}));

  action_queue_.queueTestActionQueue([this] {
    const std::set<WorkRegionId> requested_set{
      WorkRegionId::assembly,
      WorkRegionId::kitting_near_bins,
      WorkRegionId::conveyor_belt,
    };
    auto pap_robot_handle = uut_->getPickAndPlaceRobotHandle(requested_set);
    EXPECT_NE(std::nullopt, pap_robot_handle);
    EXPECT_EQ(std::set<WorkRegionId>{}, pap_robot_handle.value().resource()->supportedRegions());
  });

  EXPECT_CALL(*kitting_robot_mock_, enabled()).WillOnce(Return(true));
  EXPECT_CALL(*kitting_robot_mock_, supportedRegions())
      .WillOnce(Return(std::set<WorkRegionId>{
          WorkRegionId::assembly,
          WorkRegionId::conveyor_belt,
      }));
  EXPECT_CALL(*assembly_robot_mock_, enabled()).WillOnce(Return(true));
  EXPECT_CALL(*assembly_robot_mock_, supportedRegions())
      .WillOnce(Return(std::set<WorkRegionId>{
          WorkRegionId::assembly,
          WorkRegionId::kitting_near_bins,
          WorkRegionId::conveyor_belt,
      }));

  // This set is only to check that the correct robot handle is returned, by
  // checking that the correct mock is accessed; the value returned is not
  // important, only that it does get called.
  EXPECT_CALL(*kitting_robot_mock_, supportedRegions()).WillOnce(Return(std::set<WorkRegionId>{}));

  action_queue_.queueTestActionQueue([this] {
    const std::set<WorkRegionId> requested_set{
      WorkRegionId::assembly,
    };
    auto pap_robot_handle = uut_->getPickAndPlaceRobotHandle(requested_set);
    EXPECT_NE(std::nullopt, pap_robot_handle);
    EXPECT_EQ(std::set<WorkRegionId>{}, pap_robot_handle.value().resource()->supportedRegions());
  });

  EXPECT_CALL(*kitting_robot_mock_, enabled()).WillOnce(Return(true));
  EXPECT_CALL(*kitting_robot_mock_, supportedRegions())
      .WillOnce(Return(std::set<WorkRegionId>{
          WorkRegionId::assembly,
          WorkRegionId::conveyor_belt,
      }));
  EXPECT_CALL(*assembly_robot_mock_, enabled()).WillOnce(Return(true));
  EXPECT_CALL(*assembly_robot_mock_, supportedRegions())
      .WillOnce(Return(std::set<WorkRegionId>{
          WorkRegionId::assembly,
      }));

  // This set is only to check that the correct robot handle is returned, by
  // checking that the correct mock is accessed; the value returned is not
  // important, only that it does get called.
  EXPECT_CALL(*assembly_robot_mock_, supportedRegions()).WillOnce(Return(std::set<WorkRegionId>{}));

  action_queue_.queueTestActionQueue([this] {
    const std::set<WorkRegionId> requested_set{
      WorkRegionId::assembly,
    };
    auto pap_robot_handle = uut_->getPickAndPlaceRobotHandle(requested_set);
    EXPECT_NE(std::nullopt, pap_robot_handle);
    EXPECT_EQ(std::set<WorkRegionId>{}, pap_robot_handle.value().resource()->supportedRegions());
  });

  buildUnitUnderTest();
  ASSERT_TRUE(action_queue_.runTestActionQueue());
}

TEST_F(ResourceManagerTests, TestFindManagedLociByParent)
{
  EXPECT_CALL(*table_1_container_mock_, enabled()).WillRepeatedly(Return(true));
  EXPECT_CALL(*table_2_container_mock_, enabled()).WillRepeatedly(Return(true));
  EXPECT_CALL(*table_3_container_mock_, enabled()).WillRepeatedly(Return(true));
  buildUnitUnderTest();

  const PartId table1_parts{ part_rpump_ };
  const PartId table2_parts{ part_gpump_ };
  const PartId table3_parts{ part_rbatt_ };

  const tijmath::RelativePose3 pose1(table_1_frame_id_, table_rel_pose_11);
  const tijmath::RelativePose3 pose2(table_2_frame_id_, table_rel_pose_12);
  const tijmath::RelativePose3 pose3(table_2_frame_id_, table_rel_pose_21);
  const tijmath::RelativePose3 pose4(table_3_frame_id_, table_rel_pose_22);

  std::vector<ObservedItem> observed_models_ = {
    { QualifiedPartInfo{ table1_parts }, pose1 },
    { QualifiedPartInfo{ table2_parts }, pose2 },
    { QualifiedPartInfo{ table2_parts }, pose3 },
    { QualifiedPartInfo{ table3_parts }, pose4 },
  };
  uut_->updateSensorData(observed_models_);

  {
    auto parts_in_table1 = uut_->findManagedLociByParent(table_1_name_);
    auto parts_in_table2 = uut_->findManagedLociByParent(table_2_name_);
    auto parts_in_table3 = uut_->findManagedLociByParent(table_3_name_);

    ASSERT_EQ(1u, parts_in_table1.size());
    ASSERT_EQ(2u, parts_in_table2.size());
    ASSERT_EQ(1u, parts_in_table3.size());

    auto model_only = [](const ManagedLocus* locus_ptr) {
      auto model = locus_ptr->partId();
      return model;
    };
    ASSERT_EQ(table1_parts, model_only(parts_in_table1[0].resource()));
    ASSERT_EQ(table2_parts, model_only(parts_in_table2[0].resource()));
    ASSERT_EQ(table2_parts, model_only(parts_in_table2[1].resource()));
    ASSERT_EQ(table3_parts, model_only(parts_in_table3[0].resource()));
  }

  {
    auto locus_at_pose_2 = uut_->getManagedLocusHandleForPose(pose2);
    auto locus_at_pose_4 = uut_->getManagedLocusHandleForPose(pose4);

    EXPECT_NE(std::nullopt, locus_at_pose_2);
    EXPECT_NE(std::nullopt, locus_at_pose_4);

    auto parts_in_table1 = uut_->findManagedLociByParent(table_1_name_);
    auto parts_in_table2 = uut_->findManagedLociByParent(table_2_name_);
    auto parts_in_table3 = uut_->findManagedLociByParent(table_3_name_);

    ASSERT_EQ(1u, parts_in_table1.size());
    ASSERT_EQ(1u, parts_in_table2.size());
    ASSERT_EQ(0u, parts_in_table3.size());

    auto model_only = [](const ManagedLocus* locus_ptr) {
      auto model = locus_ptr->partId();
      return model;
    };
    ASSERT_EQ(table1_parts, model_only(parts_in_table1[0].resource()));
    ASSERT_EQ(table2_parts, model_only(parts_in_table2[0].resource()));

    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(
        pose1, parts_in_table1[0].resource()->pose(), position_tolerance_, rotation_tolerance_));
    ASSERT_TRUE(tijmath::RelativePose3::sameRelativePose3(
        pose3, parts_in_table2[0].resource()->pose(), position_tolerance_, rotation_tolerance_));
  }
}

}  // namespace

}  // namespace test

}  // namespace tijcore
