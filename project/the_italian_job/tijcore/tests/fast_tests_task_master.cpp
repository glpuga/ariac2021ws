/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// standard library
#include <functional>
#include <iostream>
#include <list>
#include <stdexcept>
#include <utility>

// gtest
#include "gtest/gtest.h"

// tijcore
#include "mocks/ModelContainerMock.hpp"
#include "mocks/PickAndPlaceRobotMock.hpp"
#include "mocks/RobotTaskMock.hpp"
#include "utils/ActionQueue.hpp"
#include <logger/logger.hpp>
#include <tijcore/perception/ResourceManager.hpp>
#include <tijcore/perception/RobotTaskFactoryInterface.hpp>
#include <tijcore/perception/StaticFrameTransformer.hpp>
#include <tijcore/perception/TaskMaster.hpp>
#include <tijcore/perception/Toolbox.hpp>

namespace tijcore {

namespace test {

namespace {

using ::testing::InSequence;
using ::testing::Invoke;
using ::testing::Return;
using ::testing::Test;

class RobotTaskFactoryFake : public RobotTaskFactoryInterface {
public:
  using Ptr = std::unique_ptr<RobotTaskFactoryFake>;
  using SharedPtr = std::shared_ptr<RobotTaskFactoryFake>;

  using SubmissionCallback = std::function<void(const std::string &)>;

  RobotTaskFactoryFake(const SubmissionCallback &submission_callback)
      : submission_callback_{submission_callback} {}

  RobotTaskInterface::Ptr getRemoveBrokenPartTask(
      ResourceManagerInterface::ManagedLocusHandle &&source_locus,
      ResourceManagerInterface::PickAndPlaceRobotHandle &&robot)
      const override {
    auto executor = [source_locus = std::move(source_locus),
                     robot = std::move(robot)]() mutable {
      auto &src = *source_locus.resource();
      auto [part_id, broken] = src.model();
      INFO("Removing {} (broken {}) from {} (@{})", part_id.codedString(),
           broken, src.pose(), src.parentName());
      auto null_place =
          ManagedLocus::CreateEmptySpace(source_locus.resource()->parentName(),
                                         source_locus.resource()->pose());
      ManagedLocus::TransferPartFromHereToThere(*source_locus.resource(),
                                                null_place);
      return RobotTaskOutcome::TASK_SUCCESS;
    };
    auto mock = std::make_unique<RobotTaskMock>();
    EXPECT_CALL(*mock, run()).WillOnce(Invoke(executor));
    return std::move(mock);
  }

  RobotTaskInterface::Ptr getPickAndPlaceTask(
      ResourceManagerInterface::ManagedLocusHandle &&source,
      ResourceManagerInterface::ManagedLocusHandle &&destination,
      ResourceManagerInterface::PickAndPlaceRobotHandle &&robot)
      const override {
    auto executor = [source = std::move(source),
                     destination = std::move(destination),
                     robot = std::move(robot)]() mutable {
      auto &src = *source.resource();
      auto &dst = *destination.resource();
      auto [part_id, broken] = src.model();
      INFO("Moving {} (broken {}) from {} (@{}) to {} (@{})",
           part_id.codedString(), broken, src.pose(), src.parentName(),
           dst.pose(), dst.parentName());
      ManagedLocus::TransferPartFromHereToThere(*source.resource(),
                                                *destination.resource());
      return RobotTaskOutcome::TASK_SUCCESS;
    };
    auto mock = std::make_unique<RobotTaskMock>();
    EXPECT_CALL(*mock, run()).WillOnce(Invoke(executor));
    return std::move(mock);
  }

  RobotTaskInterface::Ptr getSubmitKittingShipmentTask(
      ResourceManagerInterface::SubmissionTrayHandle &&tray,
      const StationId &destination_station,
      const ShipmentType &shipment_type) const override {
    if (!agv::isValid(tray.resource()->name())) {
      throw std::invalid_argument{tray.resource()->name() +
                                  " is not an agv id!"};
    }
    auto executor = [this, tray = std::move(tray), destination_station,
                     shipment_type]() mutable {
      INFO("Submitting kitting shipment {} in tray {} to station {}",
           shipment_type, tray.resource()->name(),
           tijcore::station_id::toString(destination_station));
      tray.resource()->submit();
      submission_callback_(tray.resource()->name());
      return RobotTaskOutcome::TASK_SUCCESS;
    };
    auto mock = std::make_unique<RobotTaskMock>();
    EXPECT_CALL(*mock, run()).WillOnce(Invoke(executor));
    return std::move(mock);
  }

  RobotTaskInterface::Ptr getSubmitAssemblyShipmentTask(
      ResourceManagerInterface::SubmissionTrayHandle &&tray,
      const ShipmentType &shipment_type) const override {
    if (!station_id::isValid(tray.resource()->name())) {
      throw std::invalid_argument{tray.resource()->name() +
                                  " is not a station id!"};
    }
    auto executor = [this, tray = std::move(tray), shipment_type]() mutable {
      INFO("Submitting assembly shipment {} in tray {}", shipment_type,
           tray.resource()->name());
      tray.resource()->submit();
      submission_callback_(tray.resource()->name());
      return RobotTaskOutcome::TASK_SUCCESS;
    };
    auto mock = std::make_unique<RobotTaskMock>();
    EXPECT_CALL(*mock, run()).WillOnce(Invoke(executor));
    return std::move(mock);
  }

private:
  SubmissionCallback submission_callback_;
};

class TaskMasterTests : public Test {
public:
  const double position_tolerance_{1e-3};
  const double rotation_tolerance_{1e-3};

  const CuboidVolume table_container_volume{Vector3{0, 0, -0.1},
                                            Vector3{0.9, 0.9, 0.1}};
  const CuboidVolume table_exclusion_volume{Vector3{0, 0, 0.0},
                                            Vector3{0.9, 0.9, 1.0}};

  const Pose3 table_rel_pose_11{Position::fromVector(0.22, 0.22, 0),
                                Rotation::fromQuaternion(0, 0, 0, 1)};
  const Pose3 table_rel_pose_12{Position::fromVector(0.68, 0.22, 0),
                                Rotation::fromQuaternion(1, 0, 0, 1)};
  const Pose3 table_rel_pose_21{Position::fromVector(0.22, 0.68, 0),
                                Rotation::fromQuaternion(0, 1, 0, 1)};
  const Pose3 table_rel_pose_22{Position::fromVector(0.68, 0.68, 0),
                                Rotation::fromQuaternion(0, 0, 1, 1)};

  // pumps
  const PartId red_pump_{PartTypeId::pump, PartColorId::red};
  const PartId green_pump_{PartTypeId::pump, PartColorId::green};
  const PartId blue_pump_{PartTypeId::pump, PartColorId::blue};
  // batteries
  const PartId red_battery_{PartTypeId::battery, PartColorId::red};
  const PartId green_battery_{PartTypeId::battery, PartColorId::green};
  const PartId blue_battery_{PartTypeId::battery, PartColorId::blue};
  // sensors
  const PartId red_sensor_{PartTypeId::sensor, PartColorId::red};
  const PartId green_sensor_{PartTypeId::sensor, PartColorId::green};
  const PartId blue_sensor_{PartTypeId::sensor, PartColorId::blue};
  // regulators
  const PartId red_regulator_{PartTypeId::regulator, PartColorId::red};
  const PartId green_regulator_{PartTypeId::regulator, PartColorId::green};
  const PartId blue_regulator_{PartTypeId::regulator, PartColorId::blue};

  // agv1
  const std::string agv1_name_{"agv1"};
  const std::string agv1_frame_id_{"agv1_frame"};
  const RelativePose3 agv1_pose_{"world", Position::fromVector(10, 0, 1), {}};
  ModelContainerMock::Ptr agv1_container_mock_;
  CuboidVolume agv1_container_volume_{table_container_volume};
  CuboidVolume agv1_exclusion_volume_{table_container_volume};

  // agv2
  const std::string agv2_name_{"agv2"};
  const std::string agv2_frame_id_{"agv2_frame"};
  const RelativePose3 agv2_pose_{"world", Position::fromVector(14, 0, 1), {}};
  ModelContainerMock::Ptr agv2_container_mock_;
  CuboidVolume agv2_container_volume_{table_container_volume};
  CuboidVolume agv2_exclusion_volume_{table_container_volume};

  // as1
  const std::string as1_name_{"as1"};
  const std::string as1_frame_id_{"as1_frame"};
  const RelativePose3 as1_pose_{"world", Position::fromVector(10, 1, 1), {}};
  ModelContainerMock::Ptr as1_container_mock_;
  CuboidVolume as1_container_volume_{table_container_volume};
  CuboidVolume as1_exclusion_volume_{table_container_volume};

  // as2
  const std::string as2_name_{"as2"};
  const std::string as2_frame_id_{"as2_frame"};
  const RelativePose3 as2_pose_{"world", Position::fromVector(14, 1, 1), {}};
  ModelContainerMock::Ptr as2_container_mock_;
  CuboidVolume as2_container_volume_{table_container_volume};
  CuboidVolume as2_exclusion_volume_{table_container_volume};

  // bin1
  const std::string bin1_name_{"bin1"};
  const std::string bin1_frame_id_{"bin1_frame"};
  const RelativePose3 bin1_pose_{"world", Position::fromVector(12, 0, 1), {}};
  ModelContainerMock::Ptr bin1_container_mock_;
  CuboidVolume bin1_container_volume_{table_container_volume};
  CuboidVolume bin1_exclusion_volume_{table_container_volume};

  // bin2
  const std::string bin2_name_{"bin2"};
  const std::string bin2_frame_id_{"bin2_frame"};
  const RelativePose3 bin2_pose_{"world", Position::fromVector(12, 1, 1), {}};
  ModelContainerMock::Ptr bin2_container_mock_;
  CuboidVolume bin2_container_volume_{table_container_volume};
  CuboidVolume bin2_exclusion_volume_{table_container_volume};

  // robots
  PickAndPlaceRobotMock::Ptr kitting_robot_mock_;
  PickAndPlaceRobotMock::Ptr assembly_robot_mock_;

  // toolbox stuff
  std::shared_ptr<StaticFrameTransformer> static_frame_transformer_;
  std::shared_ptr<Toolbox> toolbox_;

  // resource manager
  ResourceManagerInterface::SharedPtr resource_manager_;

  // robot task factory fake
  RobotTaskFactoryInterface::SharedPtr robot_task_factory_;

  // submitted trays info
  std::vector<std::string> submitted_trays_;

  TaskMasterInterface::Ptr uut_;

  utils::ActionQueue action_queue_;

  TaskMasterTests() {
    static_frame_transformer_ = std::make_shared<StaticFrameTransformer>(
        std::initializer_list<StaticFrameTransformer::TransformTreeLink>{
            {agv1_frame_id_, agv1_pose_},
            {agv2_frame_id_, agv2_pose_},
            {as1_frame_id_, as1_pose_},
            {as2_frame_id_, as2_pose_},
            {bin1_frame_id_, bin1_pose_},
            {bin2_frame_id_, bin2_pose_},
        });

    agv1_container_mock_ = std::make_unique<ModelContainerMock>(
        agv1_name_, agv1_frame_id_, agv1_pose_, agv1_container_volume_,
        agv1_exclusion_volume_);

    agv2_container_mock_ = std::make_unique<ModelContainerMock>(
        agv2_name_, agv2_frame_id_, agv2_pose_, agv2_container_volume_,
        agv2_exclusion_volume_);

    as1_container_mock_ = std::make_unique<ModelContainerMock>(
        as1_name_, as1_frame_id_, as1_pose_, as1_container_volume_,
        as1_exclusion_volume_);

    as2_container_mock_ = std::make_unique<ModelContainerMock>(
        as2_name_, as2_frame_id_, as2_pose_, as2_container_volume_,
        as2_exclusion_volume_);

    bin1_container_mock_ = std::make_unique<ModelContainerMock>(
        bin1_name_, bin1_frame_id_, bin1_pose_, bin1_container_volume_,
        bin1_exclusion_volume_);

    bin2_container_mock_ = std::make_unique<ModelContainerMock>(
        bin2_name_, bin2_frame_id_, bin2_pose_, bin2_container_volume_,
        bin2_exclusion_volume_);

    kitting_robot_mock_ = std::make_unique<PickAndPlaceRobotMock>();
    assembly_robot_mock_ = std::make_unique<PickAndPlaceRobotMock>();

    Toolbox::Contents contents;
    contents.frame_transformer_instance = static_frame_transformer_;
    toolbox_ = std::make_shared<Toolbox>(std::move(contents));

    EXPECT_CALL(*agv1_container_mock_, region())
        .WillRepeatedly(Return(WorkRegionId::kitting));
    EXPECT_CALL(*agv1_container_mock_, enabled()).WillRepeatedly(Return(true));
    EXPECT_CALL(*agv2_container_mock_, region())
        .WillRepeatedly(Return(WorkRegionId::kitting));
    EXPECT_CALL(*agv2_container_mock_, enabled()).WillRepeatedly(Return(true));
    EXPECT_CALL(*bin1_container_mock_, region())
        .WillRepeatedly(Return(WorkRegionId::kitting));
    EXPECT_CALL(*bin1_container_mock_, enabled()).WillRepeatedly(Return(true));
    EXPECT_CALL(*bin2_container_mock_, region())
        .WillRepeatedly(Return(WorkRegionId::kitting));
    EXPECT_CALL(*bin2_container_mock_, enabled()).WillRepeatedly(Return(true));
    EXPECT_CALL(*as1_container_mock_, region())
        .WillRepeatedly(Return(WorkRegionId::assembly));
    EXPECT_CALL(*as1_container_mock_, enabled()).WillRepeatedly(Return(true));
    EXPECT_CALL(*as2_container_mock_, region())
        .WillRepeatedly(Return(WorkRegionId::assembly));
    EXPECT_CALL(*as2_container_mock_, enabled()).WillRepeatedly(Return(true));
    EXPECT_CALL(*kitting_robot_mock_, enabled()).WillRepeatedly(Return(true));
    EXPECT_CALL(*kitting_robot_mock_, supportedRegions())
        .WillRepeatedly(Return(std::set<WorkRegionId>{
            WorkRegionId::conveyor_belt, WorkRegionId::kitting}));
    EXPECT_CALL(*assembly_robot_mock_, enabled()).WillRepeatedly(Return(true));
    EXPECT_CALL(*assembly_robot_mock_, supportedRegions())
        .WillRepeatedly(Return(std::set<WorkRegionId>{WorkRegionId::kitting,
                                                      WorkRegionId::assembly}));
  }

  void submissionCallback(const std::string &tray_name) {
    submitted_trays_.push_back(tray_name);
  }

  void buildUnitUnderTest() {
    std::vector<ModelContainerInterface::Ptr> containers_;
    containers_.emplace_back(std::move(agv1_container_mock_));
    containers_.emplace_back(std::move(agv2_container_mock_));
    containers_.emplace_back(std::move(as1_container_mock_));
    containers_.emplace_back(std::move(as2_container_mock_));
    containers_.emplace_back(std::move(bin1_container_mock_));
    containers_.emplace_back(std::move(bin2_container_mock_));

    std::vector<PickAndPlaceRobotInterface::Ptr> robots_;
    robots_.push_back(std::move(kitting_robot_mock_));
    robots_.push_back(std::move(assembly_robot_mock_));

    resource_manager_ = std::make_shared<ResourceManager>(
        toolbox_, std::move(containers_), std::move(robots_));

    robot_task_factory_ = std::make_shared<RobotTaskFactoryFake>(
        [this](const std::string &tray_name) {
          submissionCallback(tray_name);
        });

    uut_ = std::make_unique<TaskMaster>(resource_manager_, robot_task_factory_,
                                        toolbox_);
  }
};

TEST_F(TaskMasterTests, ConstructionDestruction) {
  buildUnitUnderTest();
  uut_ = nullptr;
}

class KittingOrders : public TaskMasterTests {};

TEST_F(KittingOrders, NoOrderNoActions) {
  buildUnitUnderTest();
  auto actions = uut_->run();
  ASSERT_EQ(0u, actions.size());
}

TEST_F(KittingOrders, SimpleOrder) {
  std::vector<RobotTaskInterface::Ptr> actions_list;

  action_queue_.queueTestActionQueue([&, this]() {
    std::vector<ObservedModel> observed_models_ = {
        {red_pump_, RelativePose3(bin1_frame_id_, table_rel_pose_11)},
        {red_sensor_, RelativePose3(bin1_frame_id_, table_rel_pose_22)},
        {blue_pump_, RelativePose3(bin2_frame_id_, table_rel_pose_12)},
        {blue_sensor_, RelativePose3(bin2_frame_id_, table_rel_pose_21)},
    };
    resource_manager_->updateSensorData(observed_models_);
  });

  action_queue_.queueTestActionQueue([&, this]() {
    KittingShipment shipment;
    shipment.shipment_type = "shipment1";
    shipment.agv_id = AgvId::agv1;
    shipment.station_id = StationId::as1;
    shipment.products.push_back(ProductRequest{
        red_pump_, RelativePose3(agv1_frame_id_, table_rel_pose_11)});
    shipment.products.push_back(ProductRequest{
        blue_pump_, RelativePose3(agv1_frame_id_, table_rel_pose_12)});

    Order order_0;
    order_0.order_id = OrderId{"order_0"};
    order_0.kitting_shipments.push_back(shipment);

    uut_->registerOrder(order_0);
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(2u, actions_list.size());
    for (const auto &action : actions_list) {
      action->run();
    }
    actions_list.clear();
  });

  EXPECT_CALL(*agv1_container_mock_, setEnabled(false)).Times(1);

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    EXPECT_EQ(0u, submitted_trays_.size());
    actions_list.at(0)->run();
    EXPECT_EQ(1u, submitted_trays_.size());
    EXPECT_EQ("agv1", submitted_trays_.at(submitted_trays_.size() - 1));
    actions_list.clear();
  });

  //
  // Check the final status of the world
  //

  action_queue_.queueTestActionQueue([&, this]() {
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(agv1_frame_id_, table_rel_pose_11));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(red_pump_, part_id);
      EXPECT_FALSE(broken);
    }
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(agv1_frame_id_, table_rel_pose_12));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(blue_pump_, part_id);
      EXPECT_FALSE(broken);
    }
  });

  buildUnitUnderTest();
  ASSERT_TRUE(action_queue_.runTestActionQueue());
}

TEST_F(KittingOrders, OrderWithUnwantedPiecesOnAgv) {
  std::vector<RobotTaskInterface::Ptr> actions_list;

  action_queue_.queueTestActionQueue([&, this]() {
    std::vector<ObservedModel> observed_models_ = {
        {blue_pump_, RelativePose3(bin2_frame_id_, table_rel_pose_12)},
        {red_pump_, RelativePose3(bin1_frame_id_, table_rel_pose_11)},
        {red_sensor_, RelativePose3(bin1_frame_id_, table_rel_pose_22)},
        {blue_sensor_, RelativePose3(bin2_frame_id_, table_rel_pose_21)},
        {green_regulator_, RelativePose3(agv1_frame_id_, table_rel_pose_11)},
        {blue_battery_, RelativePose3(agv1_frame_id_, table_rel_pose_22)},
    };
    resource_manager_->updateSensorData(observed_models_);
  });

  action_queue_.queueTestActionQueue([&, this]() {
    KittingShipment shipment;
    shipment.shipment_type = "shipment1";
    shipment.agv_id = AgvId::agv1;
    shipment.station_id = StationId::as1;
    shipment.products.push_back(ProductRequest{
        red_pump_, RelativePose3(agv1_frame_id_, table_rel_pose_11)});
    shipment.products.push_back(ProductRequest{
        blue_pump_, RelativePose3(agv1_frame_id_, table_rel_pose_12)});

    Order order_0;
    order_0.order_id = OrderId{"order_0"};
    order_0.kitting_shipments.push_back(shipment);

    uut_->registerOrder(order_0);
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(2u, actions_list.size());
    for (const auto &action : actions_list) {
      action->run();
    }
    actions_list.clear();
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(2u, actions_list.size());
    for (const auto &action : actions_list) {
      action->run();
    }
    actions_list.clear();
  });

  EXPECT_CALL(*agv1_container_mock_, setEnabled(false)).Times(1);

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    EXPECT_EQ(0u, submitted_trays_.size());
    actions_list.at(0)->run();
    EXPECT_EQ(1u, submitted_trays_.size());
    EXPECT_EQ("agv1", submitted_trays_.at(submitted_trays_.size() - 1));
    actions_list.clear();
  });

  //
  // Check the final status of the world
  //

  action_queue_.queueTestActionQueue([&, this]() {
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(agv1_frame_id_, table_rel_pose_11));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(red_pump_, part_id);
      EXPECT_FALSE(broken);
    }
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(agv1_frame_id_, table_rel_pose_12));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(blue_pump_, part_id);
      EXPECT_FALSE(broken);
    }
  });

  buildUnitUnderTest();
  ASSERT_TRUE(action_queue_.runTestActionQueue());
}

TEST_F(KittingOrders, OrderWithBrokenPiecesOnAgv) {
  std::vector<RobotTaskInterface::Ptr> actions_list;

  action_queue_.queueTestActionQueue([&, this]() {
    std::vector<ObservedModel> observed_models_ = {
        {red_pump_, RelativePose3(bin1_frame_id_, table_rel_pose_11)},
        {red_sensor_, RelativePose3(bin1_frame_id_, table_rel_pose_22)},
        {blue_pump_, RelativePose3(bin2_frame_id_, table_rel_pose_12)},
        {blue_sensor_, RelativePose3(bin2_frame_id_, table_rel_pose_21)},
        {red_pump_, RelativePose3(agv1_frame_id_, table_rel_pose_11), true},
        {red_battery_, RelativePose3(agv1_frame_id_, table_rel_pose_12)},
        {red_battery_, RelativePose3(agv1_frame_id_, table_rel_pose_22)},
    };
    resource_manager_->updateSensorData(observed_models_);
  });

  action_queue_.queueTestActionQueue([&, this]() {
    KittingShipment shipment;
    shipment.shipment_type = "shipment1";
    shipment.agv_id = AgvId::agv1;
    shipment.station_id = StationId::as1;
    shipment.products.push_back(ProductRequest{
        red_pump_, RelativePose3(agv1_frame_id_, table_rel_pose_11)});
    shipment.products.push_back(ProductRequest{
        blue_pump_, RelativePose3(agv1_frame_id_, table_rel_pose_12)});

    Order order_0;
    order_0.order_id = OrderId{"order_0"};
    order_0.kitting_shipments.push_back(shipment);

    uut_->registerOrder(order_0);
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(2u, actions_list.size());
    for (const auto &action : actions_list) {
      action->run();
    }
    actions_list.clear();
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(2u, actions_list.size());
    for (const auto &action : actions_list) {
      action->run();
    }
    actions_list.clear();
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    actions_list.at(0)->run();
    actions_list.clear();
  });

  EXPECT_CALL(*agv1_container_mock_, setEnabled(false)).Times(1);

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    EXPECT_EQ(0u, submitted_trays_.size());
    actions_list.at(0)->run();
    EXPECT_EQ(1u, submitted_trays_.size());
    EXPECT_EQ("agv1", submitted_trays_.at(submitted_trays_.size() - 1));
    actions_list.clear();
  });

  //
  // Check the final status of the world
  //

  action_queue_.queueTestActionQueue([&, this]() {
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(agv1_frame_id_, table_rel_pose_11));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(red_pump_, part_id);
      EXPECT_FALSE(broken);
    }
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(agv1_frame_id_, table_rel_pose_12));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(blue_pump_, part_id);
      EXPECT_FALSE(broken);
    }
  });

  buildUnitUnderTest();
  ASSERT_TRUE(action_queue_.runTestActionQueue());
}

TEST_F(KittingOrders, OrderWithOrderUpdate) {
  std::vector<RobotTaskInterface::Ptr> actions_list;

  action_queue_.queueTestActionQueue([&, this]() {
    std::vector<ObservedModel> observed_models_ = {
        {red_pump_, RelativePose3(bin1_frame_id_, table_rel_pose_11)},
        {red_sensor_, RelativePose3(bin1_frame_id_, table_rel_pose_22)},
        {blue_pump_, RelativePose3(bin2_frame_id_, table_rel_pose_12)},
        {blue_sensor_, RelativePose3(bin2_frame_id_, table_rel_pose_21)},
        {green_regulator_, RelativePose3(agv1_frame_id_, table_rel_pose_11)},
        {red_battery_, RelativePose3(agv1_frame_id_, table_rel_pose_22)},
    };
    resource_manager_->updateSensorData(observed_models_);
  });

  action_queue_.queueTestActionQueue([&, this]() {
    KittingShipment shipment;
    shipment.shipment_type = "shipment1";
    shipment.agv_id = AgvId::agv1;
    shipment.station_id = StationId::as1;
    shipment.products.push_back(ProductRequest{
        red_pump_, RelativePose3(agv1_frame_id_, table_rel_pose_11)});
    shipment.products.push_back(ProductRequest{
        blue_pump_, RelativePose3(agv1_frame_id_, table_rel_pose_12)});

    Order order_0;
    order_0.order_id = OrderId{"order_0"};
    order_0.kitting_shipments.push_back(shipment);

    uut_->registerOrder(order_0);
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(2u, actions_list.size());
    for (const auto &action : actions_list) {
      action->run();
    }
    actions_list.clear();
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(2u, actions_list.size());
    for (const auto &action : actions_list) {
      action->run();
    }
    actions_list.clear();
  });

  // At this point the order is basically ready

  action_queue_.queueTestActionQueue([&, this]() {
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(agv1_frame_id_, table_rel_pose_11));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(red_pump_, part_id);
      EXPECT_FALSE(broken);
    }
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(agv1_frame_id_, table_rel_pose_12));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(blue_pump_, part_id);
      EXPECT_FALSE(broken);
    }
  });

  // We update the order, so that now the  agv2 should be used instead

  action_queue_.queueTestActionQueue([&, this]() {
    KittingShipment shipment;
    shipment.shipment_type = "shipment1";
    shipment.agv_id = AgvId::agv2;
    shipment.station_id = StationId::as1;
    shipment.products.push_back(ProductRequest{
        red_pump_, RelativePose3(agv2_frame_id_, table_rel_pose_21)});
    shipment.products.push_back(ProductRequest{
        blue_pump_, RelativePose3(agv2_frame_id_, table_rel_pose_22)});

    Order order_0_update;
    order_0_update.order_id = OrderId{"order_0_update"};
    order_0_update.kitting_shipments.push_back(shipment);

    uut_->registerOrder(order_0_update);
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(2u, actions_list.size());
    for (const auto &action : actions_list) {
      action->run();
    }
    actions_list.clear();
  });

  EXPECT_CALL(*agv2_container_mock_, setEnabled(false)).Times(1);

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    EXPECT_EQ(0u, submitted_trays_.size());
    actions_list.at(0)->run();
    EXPECT_EQ(1u, submitted_trays_.size());
    EXPECT_EQ("agv2", submitted_trays_.at(submitted_trays_.size() - 1));
    actions_list.clear();
  });

  //
  // Check the final status of the world
  //

  action_queue_.queueTestActionQueue([&, this]() {
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(agv2_frame_id_, table_rel_pose_21));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(red_pump_, part_id);
      EXPECT_FALSE(broken);
    }
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(agv2_frame_id_, table_rel_pose_22));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(blue_pump_, part_id);
      EXPECT_FALSE(broken);
    }
  });

  buildUnitUnderTest();
  ASSERT_TRUE(action_queue_.runTestActionQueue());
}

TEST_F(KittingOrders, TwoOrdersAtTheSameTime) {
  std::vector<RobotTaskInterface::Ptr> actions_list;

  action_queue_.queueTestActionQueue([&, this]() {
    std::vector<ObservedModel> observed_models_ = {
        {red_pump_, RelativePose3(bin1_frame_id_, table_rel_pose_11)},
        {red_sensor_, RelativePose3(bin1_frame_id_, table_rel_pose_12)},
        {blue_pump_, RelativePose3(bin2_frame_id_, table_rel_pose_11)},
        {blue_sensor_, RelativePose3(bin2_frame_id_, table_rel_pose_21)},
        {red_sensor_, RelativePose3(agv1_frame_id_, table_rel_pose_21)},
        {red_battery_, RelativePose3(agv1_frame_id_, table_rel_pose_22)},
    };
    resource_manager_->updateSensorData(observed_models_);
  });

  action_queue_.queueTestActionQueue([&, this]() {
    {
      KittingShipment shipment;
      shipment.shipment_type = "shipment1";
      shipment.agv_id = AgvId::agv1;
      shipment.station_id = StationId::as1;
      shipment.products.push_back(ProductRequest{
          red_sensor_, RelativePose3(agv1_frame_id_, table_rel_pose_12)});
      shipment.products.push_back(ProductRequest{
          blue_sensor_, RelativePose3(agv1_frame_id_, table_rel_pose_21)});

      Order order_0;
      order_0.order_id = OrderId{"order_0"};
      order_0.kitting_shipments.push_back(shipment);

      uut_->registerOrder(order_0);
    }
    {
      KittingShipment shipment;
      shipment.shipment_type = "shipment1";
      shipment.agv_id = AgvId::agv2;
      shipment.station_id = StationId::as2;
      shipment.products.push_back(ProductRequest{
          red_pump_, RelativePose3(agv2_frame_id_, table_rel_pose_11)});
      shipment.products.push_back(ProductRequest{
          blue_pump_, RelativePose3(agv2_frame_id_, table_rel_pose_22)});

      Order order_1;
      order_1.order_id = OrderId{"order_1"};
      order_1.kitting_shipments.push_back(shipment);

      uut_->registerOrder(order_1);
    }
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(2u, actions_list.size());
    for (const auto &action : actions_list) {
      action->run();
    }
    actions_list.clear();
  });

  //
  // The low priority order on agv2 must be ready to submit. Check the final
  // status on agv2
  //

  EXPECT_CALL(*agv2_container_mock_, setEnabled(false)).Times(1);

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(3u, actions_list.size());
    EXPECT_EQ(0u, submitted_trays_.size());
    for (const auto &action : actions_list) {
      action->run();
    }
    EXPECT_EQ(1u, submitted_trays_.size());
    EXPECT_EQ("agv2", submitted_trays_.at(submitted_trays_.size() - 1));
    actions_list.clear();
  });

  action_queue_.queueTestActionQueue([&, this]() {
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(agv2_frame_id_, table_rel_pose_11));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(red_pump_, part_id);
      EXPECT_FALSE(broken);
    }
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(agv2_frame_id_, table_rel_pose_22));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(blue_pump_, part_id);
      EXPECT_FALSE(broken);
    }
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(2u, actions_list.size());
    for (const auto &action : actions_list) {
      action->run();
    }
    actions_list.clear();
  });

  //
  // Now the low priority order on agv1 must be ready to submit too. Check the
  // final status on agv1
  //

  EXPECT_CALL(*agv1_container_mock_, setEnabled(false)).Times(1);

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    EXPECT_EQ(1u, submitted_trays_.size());
    actions_list.at(0)->run();
    EXPECT_EQ(2u, submitted_trays_.size());
    EXPECT_EQ("agv1", submitted_trays_.at(submitted_trays_.size() - 1));
    actions_list.clear();
  });

  action_queue_.queueTestActionQueue([&, this]() {
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(agv1_frame_id_, table_rel_pose_12));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(red_sensor_, part_id);
      EXPECT_FALSE(broken);
    }
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(agv1_frame_id_, table_rel_pose_21));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(blue_sensor_, part_id);
      EXPECT_FALSE(broken);
    }
  });

  buildUnitUnderTest();
  ASSERT_TRUE(action_queue_.runTestActionQueue());
}

TEST_F(KittingOrders, NotEnoughPartsToComplete) {
  std::vector<RobotTaskInterface::Ptr> actions_list;

  action_queue_.queueTestActionQueue([&, this]() {
    std::vector<ObservedModel> observed_models_ = {
        {red_pump_, RelativePose3(bin1_frame_id_, table_rel_pose_11)},
        {red_sensor_, RelativePose3(bin1_frame_id_, table_rel_pose_22)},
        {blue_sensor_, RelativePose3(bin2_frame_id_, table_rel_pose_21)},
    };
    resource_manager_->updateSensorData(observed_models_);
  });

  action_queue_.queueTestActionQueue([&, this]() {
    KittingShipment shipment;
    shipment.shipment_type = "shipment1";
    shipment.agv_id = AgvId::agv1;
    shipment.station_id = StationId::as1;
    shipment.products.push_back(ProductRequest{
        red_pump_, RelativePose3(agv1_frame_id_, table_rel_pose_11)});
    shipment.products.push_back(ProductRequest{
        blue_pump_, RelativePose3(agv1_frame_id_, table_rel_pose_12)});

    Order order_0;
    order_0.order_id = OrderId{"order_0"};
    order_0.kitting_shipments.push_back(shipment);

    uut_->registerOrder(order_0);
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    actions_list.at(0)->run();
    actions_list.clear();
  });

  EXPECT_CALL(*agv1_container_mock_, setEnabled(false)).Times(1);

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    EXPECT_EQ(0u, submitted_trays_.size());
    actions_list.at(0)->run();
    EXPECT_EQ(1u, submitted_trays_.size());
    EXPECT_EQ("agv1", submitted_trays_.at(submitted_trays_.size() - 1));
    actions_list.clear();
  });

  //
  // Check the final status of the world
  //

  action_queue_.queueTestActionQueue([&, this]() {
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(agv1_frame_id_, table_rel_pose_11));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(red_pump_, part_id);
      EXPECT_FALSE(broken);
    }
  });

  buildUnitUnderTest();
  ASSERT_TRUE(action_queue_.runTestActionQueue());
}

class AssemblyOrders : public TaskMasterTests {};

TEST_F(AssemblyOrders, NoOrderNoActions) {
  buildUnitUnderTest();
  auto actions = uut_->run();
  ASSERT_EQ(0u, actions.size());
}

TEST_F(AssemblyOrders, SimpleOrder) {
  std::vector<RobotTaskInterface::Ptr> actions_list;

  action_queue_.queueTestActionQueue([&, this]() {
    std::vector<ObservedModel> observed_models_ = {
        {red_pump_, RelativePose3(bin1_frame_id_, table_rel_pose_11)},
        {red_sensor_, RelativePose3(bin1_frame_id_, table_rel_pose_22)},
        {blue_pump_, RelativePose3(bin2_frame_id_, table_rel_pose_12)},
        {blue_sensor_, RelativePose3(bin2_frame_id_, table_rel_pose_21)},
    };
    resource_manager_->updateSensorData(observed_models_);
  });

  action_queue_.queueTestActionQueue([&, this]() {
    AssemblyShipment shipment;
    shipment.shipment_type = "shipment1";
    shipment.station_id = StationId::as1;
    shipment.products.push_back(ProductRequest{
        red_pump_, RelativePose3(as1_frame_id_, table_rel_pose_11)});
    shipment.products.push_back(ProductRequest{
        blue_pump_, RelativePose3(as1_frame_id_, table_rel_pose_12)});

    Order order_0;
    order_0.order_id = OrderId{"order_0"};
    order_0.assembly_shipments.push_back(shipment);

    uut_->registerOrder(order_0);
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    actions_list.at(0)->run();
    actions_list.clear();
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    actions_list.at(0)->run();
    actions_list.clear();
  });

  EXPECT_CALL(*as1_container_mock_, setEnabled(false)).Times(1);

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    EXPECT_EQ(0u, submitted_trays_.size());
    actions_list.at(0)->run();
    EXPECT_EQ(1u, submitted_trays_.size());
    EXPECT_EQ("as1", submitted_trays_.at(submitted_trays_.size() - 1));
    actions_list.clear();
  });

  //
  // Check the final status of the world
  //

  action_queue_.queueTestActionQueue([&, this]() {
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(as1_frame_id_, table_rel_pose_11));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(red_pump_, part_id);
      EXPECT_FALSE(broken);
    }
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(as1_frame_id_, table_rel_pose_12));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(blue_pump_, part_id);
      EXPECT_FALSE(broken);
    }
  });

  buildUnitUnderTest();
  ASSERT_TRUE(action_queue_.runTestActionQueue());
}

TEST_F(AssemblyOrders, OrderWithUnwantedPiecesOnAgv) {
  std::vector<RobotTaskInterface::Ptr> actions_list;

  action_queue_.queueTestActionQueue([&, this]() {
    std::vector<ObservedModel> observed_models_ = {
        {blue_pump_, RelativePose3(bin2_frame_id_, table_rel_pose_12)},
        {red_pump_, RelativePose3(bin1_frame_id_, table_rel_pose_11)},
        {red_sensor_, RelativePose3(bin1_frame_id_, table_rel_pose_22)},
        {blue_sensor_, RelativePose3(bin2_frame_id_, table_rel_pose_21)},
        {green_regulator_, RelativePose3(as1_frame_id_, table_rel_pose_11)},
        {blue_battery_, RelativePose3(as1_frame_id_, table_rel_pose_22)},
    };
    resource_manager_->updateSensorData(observed_models_);
  });

  action_queue_.queueTestActionQueue([&, this]() {
    AssemblyShipment shipment;
    shipment.shipment_type = "shipment1";
    shipment.station_id = StationId::as1;
    shipment.products.push_back(ProductRequest{
        red_pump_, RelativePose3(as1_frame_id_, table_rel_pose_11)});
    shipment.products.push_back(ProductRequest{
        blue_pump_, RelativePose3(as1_frame_id_, table_rel_pose_12)});

    Order order_0;
    order_0.order_id = OrderId{"order_0"};
    order_0.assembly_shipments.push_back(shipment);

    uut_->registerOrder(order_0);
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    actions_list.at(0)->run();
    actions_list.clear();
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    actions_list.at(0)->run();
    actions_list.clear();
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    actions_list.at(0)->run();
    actions_list.clear();
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    actions_list.at(0)->run();
    actions_list.clear();
  });

  EXPECT_CALL(*as1_container_mock_, setEnabled(false)).Times(1);

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    EXPECT_EQ(0u, submitted_trays_.size());
    actions_list.at(0)->run();
    EXPECT_EQ(1u, submitted_trays_.size());
    EXPECT_EQ("as1", submitted_trays_.at(submitted_trays_.size() - 1));
    actions_list.clear();
  });

  //
  // Check the final status of the world
  //

  action_queue_.queueTestActionQueue([&, this]() {
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(as1_frame_id_, table_rel_pose_11));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(red_pump_, part_id);
      EXPECT_FALSE(broken);
    }
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(as1_frame_id_, table_rel_pose_12));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(blue_pump_, part_id);
      EXPECT_FALSE(broken);
    }
  });

  buildUnitUnderTest();
  ASSERT_TRUE(action_queue_.runTestActionQueue());
}

TEST_F(AssemblyOrders, OrderWithBrokenPiecesOnAgv) {
  std::vector<RobotTaskInterface::Ptr> actions_list;

  action_queue_.queueTestActionQueue([&, this]() {
    std::vector<ObservedModel> observed_models_ = {
        {red_pump_, RelativePose3(bin1_frame_id_, table_rel_pose_11)},
        {red_sensor_, RelativePose3(bin1_frame_id_, table_rel_pose_22)},
        {blue_pump_, RelativePose3(bin2_frame_id_, table_rel_pose_12)},
        {blue_sensor_, RelativePose3(bin2_frame_id_, table_rel_pose_21)},
        {red_pump_, RelativePose3(as1_frame_id_, table_rel_pose_11), true},
        {red_battery_, RelativePose3(as1_frame_id_, table_rel_pose_12)},
        {red_battery_, RelativePose3(as1_frame_id_, table_rel_pose_22)},
    };
    resource_manager_->updateSensorData(observed_models_);
  });

  action_queue_.queueTestActionQueue([&, this]() {
    AssemblyShipment shipment;
    shipment.shipment_type = "shipment1";
    shipment.station_id = StationId::as1;
    shipment.products.push_back(ProductRequest{
        red_pump_, RelativePose3(as1_frame_id_, table_rel_pose_11)});
    shipment.products.push_back(ProductRequest{
        blue_pump_, RelativePose3(as1_frame_id_, table_rel_pose_12)});

    Order order_0;
    order_0.order_id = OrderId{"order_0"};
    order_0.assembly_shipments.push_back(shipment);

    uut_->registerOrder(order_0);
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    actions_list.at(0)->run();
    actions_list.clear();
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    actions_list.at(0)->run();
    actions_list.clear();
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    actions_list.at(0)->run();
    actions_list.clear();
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    actions_list.at(0)->run();
    actions_list.clear();
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    actions_list.at(0)->run();
    actions_list.clear();
  });

  EXPECT_CALL(*as1_container_mock_, setEnabled(false)).Times(1);

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    EXPECT_EQ(0u, submitted_trays_.size());
    actions_list.at(0)->run();
    EXPECT_EQ(1u, submitted_trays_.size());
    EXPECT_EQ("as1", submitted_trays_.at(submitted_trays_.size() - 1));
    actions_list.clear();
  });

  //
  // Check the final status of the world
  //

  action_queue_.queueTestActionQueue([&, this]() {
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(as1_frame_id_, table_rel_pose_11));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(red_pump_, part_id);
      EXPECT_FALSE(broken);
    }
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(as1_frame_id_, table_rel_pose_12));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(blue_pump_, part_id);
      EXPECT_FALSE(broken);
    }
  });

  buildUnitUnderTest();
  ASSERT_TRUE(action_queue_.runTestActionQueue());
}

TEST_F(AssemblyOrders, OrderWithOrderUpdate) {
  std::vector<RobotTaskInterface::Ptr> actions_list;

  action_queue_.queueTestActionQueue([&, this]() {
    std::vector<ObservedModel> observed_models_ = {
        {red_pump_, RelativePose3(bin1_frame_id_, table_rel_pose_11)},
        {red_sensor_, RelativePose3(bin1_frame_id_, table_rel_pose_22)},
        {blue_pump_, RelativePose3(bin2_frame_id_, table_rel_pose_12)},
        {blue_sensor_, RelativePose3(bin2_frame_id_, table_rel_pose_21)},
        {green_regulator_, RelativePose3(as1_frame_id_, table_rel_pose_11)},
        {red_battery_, RelativePose3(as1_frame_id_, table_rel_pose_22)},
    };
    resource_manager_->updateSensorData(observed_models_);
  });

  action_queue_.queueTestActionQueue([&, this]() {
    AssemblyShipment shipment;
    shipment.shipment_type = "shipment1";
    shipment.station_id = StationId::as1;
    shipment.products.push_back(ProductRequest{
        red_pump_, RelativePose3(as1_frame_id_, table_rel_pose_11)});
    shipment.products.push_back(ProductRequest{
        blue_pump_, RelativePose3(as1_frame_id_, table_rel_pose_12)});

    Order order_0;
    order_0.order_id = OrderId{"order_0"};
    order_0.assembly_shipments.push_back(shipment);

    uut_->registerOrder(order_0);
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    actions_list.at(0)->run();
    actions_list.clear();
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    actions_list.at(0)->run();
    actions_list.clear();
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    actions_list.at(0)->run();
    actions_list.clear();
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    actions_list.at(0)->run();
    actions_list.clear();
  });

  // At this point the order is basically ready

  action_queue_.queueTestActionQueue([&, this]() {
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(as1_frame_id_, table_rel_pose_11));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(red_pump_, part_id);
      EXPECT_FALSE(broken);
    }
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(as1_frame_id_, table_rel_pose_12));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(blue_pump_, part_id);
      EXPECT_FALSE(broken);
    }
  });

  // We update the order, so that now the  agv2 should be used instead

  action_queue_.queueTestActionQueue([&, this]() {
    AssemblyShipment shipment;
    shipment.shipment_type = "shipment1";
    shipment.station_id = StationId::as2;
    shipment.products.push_back(ProductRequest{
        red_pump_, RelativePose3(as2_frame_id_, table_rel_pose_21)});
    shipment.products.push_back(ProductRequest{
        blue_pump_, RelativePose3(as2_frame_id_, table_rel_pose_22)});

    Order order_0_update;
    order_0_update.order_id = OrderId{"order_0_update"};
    order_0_update.assembly_shipments.push_back(shipment);

    uut_->registerOrder(order_0_update);
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    actions_list.at(0)->run();
    actions_list.clear();
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    actions_list.at(0)->run();
    actions_list.clear();
  });

  EXPECT_CALL(*as2_container_mock_, setEnabled(false)).Times(1);

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    EXPECT_EQ(0u, submitted_trays_.size());
    actions_list.at(0)->run();
    EXPECT_EQ(1u, submitted_trays_.size());
    EXPECT_EQ("as2", submitted_trays_.at(submitted_trays_.size() - 1));
    actions_list.clear();
  });

  //
  // Check the final status of the world
  //

  action_queue_.queueTestActionQueue([&, this]() {
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(as2_frame_id_, table_rel_pose_21));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(red_pump_, part_id);
      EXPECT_FALSE(broken);
    }
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(as2_frame_id_, table_rel_pose_22));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(blue_pump_, part_id);
      EXPECT_FALSE(broken);
    }
  });

  buildUnitUnderTest();
  ASSERT_TRUE(action_queue_.runTestActionQueue());
}

TEST_F(AssemblyOrders, TwoOrdersAtTheSameTime) {
  std::vector<RobotTaskInterface::Ptr> actions_list;

  action_queue_.queueTestActionQueue([&, this]() {
    std::vector<ObservedModel> observed_models_ = {
        {red_pump_, RelativePose3(bin1_frame_id_, table_rel_pose_11)},
        {red_sensor_, RelativePose3(bin1_frame_id_, table_rel_pose_12)},
        {blue_pump_, RelativePose3(bin2_frame_id_, table_rel_pose_11)},
        {blue_sensor_, RelativePose3(bin2_frame_id_, table_rel_pose_21)},
        {red_sensor_, RelativePose3(as1_frame_id_, table_rel_pose_21)},
        {red_battery_, RelativePose3(as1_frame_id_, table_rel_pose_22)},
    };
    resource_manager_->updateSensorData(observed_models_);
  });

  action_queue_.queueTestActionQueue([&, this]() {
    {
      AssemblyShipment shipment;
      shipment.shipment_type = "shipment1";
      shipment.station_id = StationId::as1;
      shipment.products.push_back(ProductRequest{
          red_sensor_, RelativePose3(as1_frame_id_, table_rel_pose_12)});
      shipment.products.push_back(ProductRequest{
          blue_sensor_, RelativePose3(as1_frame_id_, table_rel_pose_21)});

      Order order_0;
      order_0.order_id = OrderId{"order_0"};
      order_0.assembly_shipments.push_back(shipment);

      uut_->registerOrder(order_0);
    }
    {
      AssemblyShipment shipment;
      shipment.shipment_type = "shipment1";
      shipment.station_id = StationId::as2;
      shipment.products.push_back(ProductRequest{
          red_pump_, RelativePose3(as2_frame_id_, table_rel_pose_11)});
      shipment.products.push_back(ProductRequest{
          blue_pump_, RelativePose3(as2_frame_id_, table_rel_pose_22)});

      Order order_1;
      order_1.order_id = OrderId{"order_1"};
      order_1.assembly_shipments.push_back(shipment);

      uut_->registerOrder(order_1);
    }
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    actions_list.at(0)->run();
    actions_list.clear();
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    actions_list.at(0)->run();
    actions_list.clear();
  });

  //
  // The low priority order on agv2 must be ready to submit. Check the final
  // status on agv2
  //

  EXPECT_CALL(*as2_container_mock_, setEnabled(false)).Times(1);

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(2u, actions_list.size());
    EXPECT_EQ(0u, submitted_trays_.size());
    for (const auto &action : actions_list) {
      action->run();
    }
    EXPECT_EQ(1u, submitted_trays_.size());
    EXPECT_EQ("as2", submitted_trays_.at(submitted_trays_.size() - 1));
    actions_list.clear();
  });

  action_queue_.queueTestActionQueue([&, this]() {
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(as2_frame_id_, table_rel_pose_11));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(red_pump_, part_id);
      EXPECT_FALSE(broken);
    }
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(as2_frame_id_, table_rel_pose_22));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(blue_pump_, part_id);
      EXPECT_FALSE(broken);
    }
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    actions_list.at(0)->run();
    actions_list.clear();
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    actions_list.at(0)->run();
    actions_list.clear();
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    actions_list.at(0)->run();
    actions_list.clear();
  });

  //
  // Now the low priority order on agv1 must be ready to submit too. Check the
  // final status on agv1
  //

  EXPECT_CALL(*as1_container_mock_, setEnabled(false)).Times(1);

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    EXPECT_EQ(1u, submitted_trays_.size());
    actions_list.at(0)->run();
    EXPECT_EQ(2u, submitted_trays_.size());
    EXPECT_EQ("as1", submitted_trays_.at(submitted_trays_.size() - 1));
    actions_list.clear();
  });

  action_queue_.queueTestActionQueue([&, this]() {
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(as1_frame_id_, table_rel_pose_12));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(red_sensor_, part_id);
      EXPECT_FALSE(broken);
    }
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(as1_frame_id_, table_rel_pose_21));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(blue_sensor_, part_id);
      EXPECT_FALSE(broken);
    }
  });

  buildUnitUnderTest();
  ASSERT_TRUE(action_queue_.runTestActionQueue());
}

TEST_F(AssemblyOrders, NotEnoughPartsToComplete) {
  std::vector<RobotTaskInterface::Ptr> actions_list;

  action_queue_.queueTestActionQueue([&, this]() {
    std::vector<ObservedModel> observed_models_ = {
        {red_pump_, RelativePose3(bin1_frame_id_, table_rel_pose_11)},
        {red_sensor_, RelativePose3(bin1_frame_id_, table_rel_pose_22)},
        {blue_sensor_, RelativePose3(bin2_frame_id_, table_rel_pose_21)},
    };
    resource_manager_->updateSensorData(observed_models_);
  });

  action_queue_.queueTestActionQueue([&, this]() {
    AssemblyShipment shipment;
    shipment.shipment_type = "shipment1";
    shipment.station_id = StationId::as1;
    shipment.products.push_back(ProductRequest{
        red_pump_, RelativePose3(as1_frame_id_, table_rel_pose_11)});
    shipment.products.push_back(ProductRequest{
        blue_pump_, RelativePose3(as1_frame_id_, table_rel_pose_12)});

    Order order_0;
    order_0.order_id = OrderId{"order_0"};
    order_0.assembly_shipments.push_back(shipment);

    uut_->registerOrder(order_0);
  });

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    actions_list.at(0)->run();
    actions_list.clear();
  });

  EXPECT_CALL(*as1_container_mock_, setEnabled(false)).Times(1);

  action_queue_.queueTestActionQueue([&, this]() {
    actions_list = uut_->run();
    EXPECT_EQ(1u, actions_list.size());
    EXPECT_EQ(0u, submitted_trays_.size());
    actions_list.at(0)->run();
    EXPECT_EQ(1u, submitted_trays_.size());
    EXPECT_EQ("as1", submitted_trays_.at(submitted_trays_.size() - 1));
    actions_list.clear();
  });

  //
  // Check the final status of the world
  //

  action_queue_.queueTestActionQueue([&, this]() {
    {
      auto handle = resource_manager_->getManagedLocusHandleForPose(
          RelativePose3(as1_frame_id_, table_rel_pose_11));
      auto [part_id, broken] = handle->resource()->model();
      EXPECT_EQ(red_pump_, part_id);
      EXPECT_FALSE(broken);
    }
  });

  buildUnitUnderTest();
  ASSERT_TRUE(action_queue_.runTestActionQueue());
}

} // namespace

} // namespace test

} // namespace tijcore
