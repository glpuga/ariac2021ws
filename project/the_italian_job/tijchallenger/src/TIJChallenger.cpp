/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

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
#include <tijcore/perception/AgvModelContainer.hpp>
#include <tijcore/perception/AssemblyStationModelContainer.hpp>
#include <tijcore/perception/BinModelContainer.hpp>
#include <tijcore/perception/ConveyorBeltModelContainer.hpp>
#include <tijcore/perception/ModelPerceptionMixer.hpp>
#include <tijcore/perception/ResourceManager.hpp>
#include <tijcore/perception/RobotTaskFactory.hpp>
#include <tijcore/perception/TaskDriver.hpp>
#include <tijcore/perception/TaskMaster.hpp>
#include <tijcore/tasks/RobotTaskGroupRunner.hpp>
#include <tijlogger/logger.hpp>
#include <tijros/ConveyorBeltSurfaceFrameBroadcaster.hpp>
#include <tijros/LogicalCameraModelPerception.hpp>
#include <tijros/PickAndPlaceAssemblyRobot.hpp>
#include <tijros/PickAndPlaceKittingRobot.hpp>
#include <tijros/QualityControlSensorModelPerception.hpp>
#include <tijros/ROSFrameTransformer.hpp>
#include <tijros/ROSProcessManagement.hpp>
#include <tijros/ROSRobotActuators.hpp>

namespace tijchallenger
{
namespace
{
std::chrono::seconds system_load_delay_{ 5 };
}

TIJChallenger::TIJChallenger()
{
  INFO("Setting up challenger");

  INFO(" - Instantiating scene configuration");
  config_ = std::make_shared<SceneConfigReader>();

  INFO(" - Creating conveyor belt frame broadcasters");
  containers_ = createConveyorBeltTransformBroadcasters();

  INFO(" - Creating Toolbox");
  toolbox_ = createToolbox();

  INFO(" - Creating ResourceManager");
  auto resource_manager =
      std::make_shared<tijcore::ResourceManager>(toolbox_, config_->getListOfSharedAccessSpaceDescriptions(),
                                                 createModelContainers(toolbox_), createPickAndPlaceRobots(toolbox_));

  INFO(" - Creating RobotTaskFactory");
  auto task_master = std::make_unique<tijcore::TaskMaster>(
      resource_manager, std::make_unique<tijcore::RobotTaskFactory>(resource_manager, toolbox_), toolbox_);

  INFO(" - Creating TaskDriver");
  task_driver_ =
      std::make_unique<tijcore::TaskDriver>(std::move(task_master), std::make_unique<tijcore::RobotTaskGroupRunner>(),
                                            resource_manager, createModelPerceptionMixer(), toolbox_);

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
    cameras.emplace_back(std::make_unique<tijros::LogicalCameraModelPerception>(nh_, item.name));
  }

  INFO(" - Loading quality control sensors information");
  for (const auto& item : config_->getListOfQualityControlSensors())
  {
    INFO("   - {} @ {}", item.name, item.frame_id);
    cameras.emplace_back(std::make_unique<tijros::QualityControlSensorModelPerception>(nh_, item.name));
  }

  return std::make_unique<tijcore::ModelPerceptionMixer>(std::move(cameras));
}

tijcore::Toolbox::SharedPtr TIJChallenger::createToolbox() const
{
  tijcore::Toolbox::Contents contents;
  contents.frame_transformer_instance = std::make_shared<tijros::ROSFrameTransformer>();
  contents.robot_actuator_instance = std::make_shared<tijros::ROSRobotActuators>(nh_);
  contents.process_manager_instance = std::make_shared<tijros::ROSProcessManagement>(nh_);
  contents.scene_config_reader_instance = config_;
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
        std::make_unique<tijcore::AgvModelContainer>(item.name, item.frame_id, item.shared_access_space_id, toolbox_));
  }

  INFO(" - Loading assembly station data");
  for (const auto& item : config_->getListOfAssemblyStations())
  {
    INFO("   - {} @ {}", item.name, item.frame_id);
    containers.emplace_back(std::make_unique<tijcore::AssemblyStationModelContainer>(
        item.name, item.frame_id, item.shared_access_space_id, toolbox_));
  }

  INFO(" - Loading bin data");
  for (const auto& item : config_->getListOfBins())
  {
    INFO("   - {} @ {} ({})", item.name, item.frame_id, item.work_region);
    containers.emplace_back(std::make_unique<tijcore::BinModelContainer>(item.name, item.frame_id, item.work_region,
                                                                         item.shared_access_space_id));
  }

  INFO(" - Loading conveyor belts data");
  for (const auto& item : config_->getListOfConveyorBelts())
  {
    INFO("   - {} @ {}/{} (speed {})", item.name, item.container_frame_id, item.surface_frame_id,
         item.meters_per_second);
    containers.emplace_back(std::make_unique<tijcore::ConveyorBeltModelContainer>(
        item.name, item.container_frame_id, item.surface_frame_id, item.shared_access_space_id));
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

std::vector<tijcore::PickAndPlaceRobotInterface::Ptr>
TIJChallenger::createPickAndPlaceRobots(const tijcore::Toolbox::SharedPtr& toolbox_) const
{
  INFO(" - Creating robot puppeteers");
  std::vector<tijcore::PickAndPlaceRobotInterface::Ptr> robots;
  robots.emplace_back(std::make_unique<tijros::PickAndPlaceKittingRobot>(toolbox_));
  robots.emplace_back(std::make_unique<tijros::PickAndPlaceAssemblyRobot>(toolbox_));
  return robots;
}

}  // namespace tijchallenger
