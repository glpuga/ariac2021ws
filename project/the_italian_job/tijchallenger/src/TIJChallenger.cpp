/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// standard library
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

// ros
#include <ros/ros.h>

// project
#include <tijchallenger/SceneConfigReader.hpp>
#include <tijchallenger/TIJChallenger.hpp>
#include <tijcore/containers/AgvModelContainer.hpp>
#include <tijcore/containers/AssemblyStationModelContainer.hpp>
#include <tijcore/containers/BinModelContainer.hpp>
#include <tijcore/containers/ConveyorBeltModelContainer.hpp>
#include <tijcore/containers/TableModelContainer.hpp>
#include <tijcore/coremodels/ModelPerceptionMixer.hpp>
#include <tijcore/coremodels/ModelPerceptionSpatialFilter.hpp>
#include <tijcore/resources/ResourceManager.hpp>
#include <tijcore/resources/SpatialMutualExclusionManager.hpp>
#include <tijcore/tasking/BTRobotTaskFactory.hpp>
#include <tijcore/tasking/RobotTaskGroupRunner.hpp>
#include <tijcore/tasking/TaskDispatcher.hpp>
#include <tijcore/tasking/TaskDriver.hpp>
#include <tijcore/utils/BlindVolumeTracker.hpp>
#include <tijlogger/logger.hpp>
#include <tijros/ConveyorBeltSurfaceFrameBroadcaster.hpp>
#include <tijros/LogicalCameraModelPerception.hpp>
#include <tijros/PickAndPlaceAssemblyRobot.hpp>
#include <tijros/PickAndPlaceKittingRobot.hpp>
#include <tijros/PickAndPlaceRobotMovements.hpp>
#include <tijros/QualityControlSensorModelPerception.hpp>
#include <tijros/ROSFrameTransformer.hpp>
#include <tijros/ROSProcessManagement.hpp>
#include <tijros/ROSRobotActuators.hpp>

namespace tijchallenger
{
namespace
{
std::chrono::seconds system_load_delay_{ 5 };

ros::Duration camera_retention_interval_{ 1.0 };
}  // namespace

TIJChallenger::TIJChallenger()
{
  std::string behavior_file_path;
  ros::param::param<std::string>("/challenger_behavior_file", behavior_file_path, "behavior.xml");
  INFO(" - Behavior file: {}", behavior_file_path);

  INFO("Setting up challenger");

  INFO(" - Instantiating scene configuration");
  config_ = std::make_shared<SceneConfigReader>();

  INFO(" - Creating Toolbox");
  toolbox_ = createToolbox();

  INFO(" - Creating ResourceManager");
  auto resource_manager = std::make_shared<tijcore::ResourceManager>(
      toolbox_, createModelContainers(toolbox_), createPickAndPlaceRobots(toolbox_));

  INFO(" - Creating RobotTaskFactory");
  auto task_master = std::make_unique<tijcore::TaskDispatcher>(
      resource_manager,
      std::make_unique<tijcore::BTRobotTaskFactory>(behavior_file_path, resource_manager, toolbox_),
      toolbox_);

  INFO(" - Creating TaskDriver");
  task_driver_ = std::make_unique<tijcore::TaskDriver>(
      std::move(task_master), std::make_unique<tijcore::RobotTaskGroupRunner>(), resource_manager,
      createModelPerceptionMixer(), toolbox_);

  INFO("Setup complete.");
}

void TIJChallenger::run()
{
  INFO("Starting spinner...");
  auto async_future = std::async(std::launch::async, [this] {
    INFO("Sleeping while the rest of the system catches up...");
    std::this_thread::sleep_for(system_load_delay_);
    WARNING("Starting competition mode...");
    toolbox_->getProcessManager()->startCompetition();
    WARNING("Done!");
  });
  ros::spin();

  async_future.wait();
}

tijcore::ModelPerceptionInterface::Ptr TIJChallenger::createModelPerceptionMixer() const
{
  std::vector<tijcore::ModelPerceptionInterface::Ptr> cameras;

  INFO(" - Loading logical cameras information");
  for (const auto& item : config_->getListOfLogicalCameras())
  {
    INFO("   - {} @ {}", item.name, item.frame_id);
    cameras.emplace_back(std::make_unique<tijros::LogicalCameraModelPerception>(
        nh_, item.name, camera_retention_interval_));
  }

  INFO(" - Loading quality control sensors information");
  for (const auto& item : config_->getListOfQualityControlSensors())
  {
    INFO("   - {} @ {}", item.name, item.frame_id);
    cameras.emplace_back(std::make_unique<tijros::QualityControlSensorModelPerception>(
        nh_, item.name, camera_retention_interval_));
  }

  auto perception_mixer = std::make_unique<tijcore::ModelPerceptionMixer>(std::move(cameras));

  auto perception_spatial_filter = std::make_unique<tijcore::ModelPerceptionSpatialFilter>(
      toolbox_, std::move(perception_mixer));

  perception_spatial_filter->addBlindVolumeTracker(std::make_unique<tijcore::BlindVolumeTracker>(
      tijmath::RelativePose3{
          "vacuum_gripper_link", tijmath::Position::fromVector(0.0, 0.0, -0.05), {} },
      0.1));
  perception_spatial_filter->addBlindVolumeTracker(std::make_unique<tijcore::BlindVolumeTracker>(
      tijmath::RelativePose3{
          "gantry_arm_vacuum_gripper_link", tijmath::Position::fromVector(0.0, 0.0, -0.05), {} },
      0.1));

  return std::move(perception_spatial_filter);
}

tijcore::Toolbox::SharedPtr TIJChallenger::createToolbox() const
{
  tijcore::Toolbox::Contents contents;
  contents.frame_transformer_instance = std::make_shared<tijros::ROSFrameTransformer>();
  contents.robot_actuator_instance = std::make_shared<tijros::ROSRobotActuators>(nh_);
  contents.process_manager_instance = std::make_shared<tijros::ROSProcessManagement>(nh_);
  contents.scene_config_reader_instance = config_;
  contents.spatial_mutual_exclusion_manager =
      std::make_shared<tijcore::SpatialMutualExclusionManager>(config_->getWorldFrameId(),
                                                               contents.frame_transformer_instance);
  return std::make_shared<tijcore::Toolbox>(std::move(contents));
}

std::vector<tijcore::ModelContainerInterface::Ptr>
TIJChallenger::createModelContainers(const tijcore::Toolbox::SharedPtr& toolbox_) const
{
  std::vector<tijcore::ModelContainerInterface::Ptr> containers;
  INFO(" - Loading agv tray data");
  for (const auto& item : config_->getListOfAgvs())
  {
    INFO("   - {} @ {}", item.name, item.frame_id);
    containers.emplace_back(
        std::make_unique<tijcore::AgvModelContainer>(item.name, item.frame_id, toolbox_));
  }

  INFO(" - Loading assembly station data");
  for (const auto& item : config_->getListOfAssemblyStations())
  {
    INFO("   - {} @ {}", item.name, item.frame_id);
    containers.emplace_back(std::make_unique<tijcore::AssemblyStationModelContainer>(
        item.name, item.frame_id, toolbox_));
  }

  INFO(" - Loading bin data");
  for (const auto& item : config_->getListOfBins())
  {
    INFO("   - {} @ {} ({})", item.name, item.frame_id);
    containers.emplace_back(std::make_unique<tijcore::BinModelContainer>(item.name, item.frame_id));
  }

  INFO(" - Loading table data");
  for (const auto& item : config_->getListOfTables())
  {
    INFO("   - {} @ {} ({})", item.name, item.frame_id);
    containers.emplace_back(
        std::make_unique<tijcore::TableModelContainer>(item.name, item.frame_id));
  }

  INFO(" - Loading conveyor belts data");
  for (const auto& item : config_->getListOfConveyorBelts())
  {
    INFO("   - {} @ {}/{} (speed {})", item.name, item.container_frame_id, item.surface_frame_id,
         item.meters_per_second);
    containers.emplace_back(std::make_unique<tijcore::ConveyorBeltModelContainer>(
        item.name, item.container_frame_id, item.surface_frame_id));
  }

  return containers;
}

std::vector<tijros::ConveyorBeltSurfaceFrameBroadcaster::Ptr>
TIJChallenger::createConveyorBeltTransformBroadcasters() const
{
  std::vector<tijros::ConveyorBeltSurfaceFrameBroadcaster::Ptr> containers;
  INFO(" - Loading conveyor belts surface transform broadcaster");
  for (const auto& item : config_->getListOfConveyorBelts())
  {
    INFO("   - {} @ {}/{} (speed {})", item.name, item.container_frame_id, item.surface_frame_id,
         item.meters_per_second);
    containers.emplace_back(std::make_unique<tijros::ConveyorBeltSurfaceFrameBroadcaster>(
        nh_, item.container_frame_id, item.surface_frame_id, item.meters_per_second));
  }

  return containers;
}

std::vector<tijcore::PickAndPlaceRobotMovementsInterface::Ptr>
TIJChallenger::createPickAndPlaceRobots(const tijcore::Toolbox::SharedPtr& toolbox_) const
{
  INFO(" - Creating robot puppeteers");
  std::vector<tijcore::PickAndPlaceRobotMovementsInterface::Ptr> robots;
  robots.emplace_back(std::make_unique<tijros::PickAndPlaceRobotMovements>(
      std::make_unique<tijros::PickAndPlaceKittingRobot>(toolbox_), toolbox_));
  robots.emplace_back(std::make_unique<tijros::PickAndPlaceRobotMovements>(
      std::make_unique<tijros::PickAndPlaceAssemblyRobot>(toolbox_), toolbox_));
  return robots;
}

}  // namespace tijchallenger
