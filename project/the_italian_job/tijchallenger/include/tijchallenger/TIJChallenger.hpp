/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>
#include <utility>
#include <vector>

// ros
#include <ros/ros.h>

// project
#include <tijcore/abstractions/HumanMonitorServiceInterface.hpp>
#include <tijcore/abstractions/ModelContainerInterface.hpp>
#include <tijcore/abstractions/ModelPerceptionInterface.hpp>
#include <tijcore/abstractions/PickAndPlaceRobotMovementsInterface.hpp>
#include <tijcore/abstractions/SceneConfigReaderInterface.hpp>
#include <tijcore/coremodels/Toolbox.hpp>
#include <tijcore/tasking/TaskDriver.hpp>
#include <tijros/ConveyorBeltSurfaceFrameBroadcaster.hpp>

namespace tijchallenger
{
class TIJChallenger
{
public:
  TIJChallenger();

  void run();

private:
  ros::NodeHandle nh_;
  tijcore::SceneConfigReaderInterface::SharedPtr config_;

  tijcore::Toolbox::SharedPtr toolbox_;
  tijcore::TaskDriver::Ptr task_driver_;

  std::vector<tijcore::HumanMonitorServiceInterface::Ptr> human_proximity_services_;

  tijcore::ModelPerceptionInterface::Ptr createFilteredModelPerceptionMixer() const;

  tijcore::ModelPerceptionInterface::Ptr createUnfilteredInputPerceptionChain() const;

  tijcore::Toolbox::SharedPtr createToolbox() const;

  std::vector<tijcore::HumanMonitorServiceInterface::Ptr> createHumanProximityServices() const;

  std::vector<tijcore::ModelContainerInterface::Ptr>
  createModelContainers(const tijcore::Toolbox::SharedPtr& toolbox) const;

  std::vector<tijros::ConveyorBeltSurfaceFrameBroadcaster::Ptr>
  createConveyorBeltTransformBroadcasters() const;

  std::vector<tijcore::PickAndPlaceRobotMovementsInterface::Ptr>
  createPickAndPlaceRobots(const tijcore::Toolbox::SharedPtr& toolbox) const;
};

}  // namespace tijchallenger
