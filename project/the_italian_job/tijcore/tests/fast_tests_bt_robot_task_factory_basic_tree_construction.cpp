/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <memory>
#include <string>
#include <utility>

// gtest
#include "gtest/gtest.h"

// mocks
#include "mocks/PickAndPlaceRobotMovementsMock.hpp"
#include "mocks/ResourceManagerMock.hpp"
#include "mocks/RobotActuatorsMock.hpp"

// tijcore
#include <tijcore/coremodels/Toolbox.hpp>
#include <tijcore/tasking/BTRobotTaskFactory.hpp>

namespace tijcore
{
namespace test
{
namespace
{
using ::testing::Test;

struct BTTestFixtureBase : public Test
{
  const std::string behavior_file_path_{ "../behavior/behavior.xml" };

  std::shared_ptr<ResourceManagerMock> resource_manager_mock_;
  std::shared_ptr<RobotActuatorsMock> robot_actuators_mock_;
  Toolbox::SharedPtr toolbox_;

  BTTestFixtureBase()
  {
    resource_manager_mock_ = std::make_shared<ResourceManagerMock>();
    robot_actuators_mock_ = std::make_shared<RobotActuatorsMock>();

    Toolbox::Contents contents;
    contents.robot_actuator_instance = robot_actuators_mock_;

    toolbox_ = std::make_shared<Toolbox>(std::move(contents));
  }
};

TEST_F(BTTestFixtureBase, BasicConstruct)
{
  std::make_unique<BTRobotTaskFactory>(behavior_file_path_, resource_manager_mock_, toolbox_);
}

struct BTTestFixtureLoaded : public BTTestFixtureBase
{
  std::shared_ptr<PickAndPlaceRobotMovementsMock> robot_mock_;
  BTRobotTaskFactory::Ptr task_factory_;

  BTTestFixtureLoaded()
  {
    robot_mock_ = std::make_shared<PickAndPlaceRobotMovementsMock>();

    // this is because during destruction the halt() methods of async nodes get called and they call
    // this in turn
    EXPECT_CALL(*robot_mock_, abortCurrentAction()).Times(testing::AnyNumber());

    task_factory_ =
        std::make_unique<BTRobotTaskFactory>(behavior_file_path_, resource_manager_mock_, toolbox_);
  }
};

TEST_F(BTTestFixtureLoaded, BuildGetRemoveBrokenPartTask)
{
  ResourceManagerInterface::ManagedLocusHandle source_locus{ std::make_unique<ManagedLocus>(
      ManagedLocus::CreatePartLocus("parent", {}, PartId::UnkownPartId, false)) };
  ResourceManagerInterface::PickAndPlaceRobotHandle robot{ std::move(robot_mock_) };
  auto uut = task_factory_->getRemoveBrokenPartTask(std::move(source_locus), std::move(robot));
}

TEST_F(BTTestFixtureLoaded, BuildGetPickAndPlacePartTask)
{
  ResourceManagerInterface::ManagedLocusHandle src_locus{ std::make_unique<ManagedLocus>(
      ManagedLocus::CreatePartLocus("parent", {}, PartId::UnkownPartId, false)) };
  ResourceManagerInterface::ManagedLocusHandle dst_locus{ std::make_unique<ManagedLocus>(
      ManagedLocus::CreateEmptyLocus("parent", {})) };
  ResourceManagerInterface::PickAndPlaceRobotHandle robot{ std::move(robot_mock_) };
  auto uut = task_factory_->getPickAndPlacePartTask(std::move(src_locus), std::move(dst_locus),
                                                    std::move(robot));
}

TEST_F(BTTestFixtureLoaded, BuildGetPickAndPlaceMovableTrayTask)
{
  ResourceManagerInterface::ManagedLocusHandle src_locus{ std::make_unique<ManagedLocus>(
      ManagedLocus::CreateMovableTrayLocus("parent", {}, MovableTrayId::movable_tray_dark_wood)) };
  ResourceManagerInterface::ManagedLocusHandle dst_locus{ std::make_unique<ManagedLocus>(
      ManagedLocus::CreateEmptyLocus("parent", {})) };
  ResourceManagerInterface::PickAndPlaceRobotHandle robot{ std::move(robot_mock_) };

  auto uut = task_factory_->getPickAndPlaceMovableTrayTask(std::move(src_locus),
                                                           std::move(dst_locus), std::move(robot));
}

}  // namespace

}  // namespace test

}  // namespace tijcore
