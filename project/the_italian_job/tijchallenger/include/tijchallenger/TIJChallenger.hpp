/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <vector>

// ros
#include <ros/ros.h>

// project
#include <tijcore/perception/ModelContainerInterface.hpp>
#include <tijcore/perception/ModelPerceptionInterface.hpp>
#include <tijcore/perception/PickAndPlaceRobotInterface.hpp>
#include <tijcore/perception/SceneConfigReaderInterface.hpp>
#include <tijcore/perception/TaskDriver.hpp>
#include <tijcore/perception/Toolbox.hpp>
#include <tijros/ConveyorBeltSurfaceFrameBroadcaster.hpp>

namespace tijchallenger {

class TIJChallenger {
public:
  TIJChallenger();

  void run();

private:
  ros::NodeHandle nh_;
  tijcore::SceneConfigReaderInterface::SharedPtr config_;

  std::vector<tijros::ConveyorBeltSurfaceFrameBroadcaster::Ptr> containers_;

  tijcore::Toolbox::SharedPtr toolbox_;
  tijcore::TaskDriver::Ptr task_driver_;

  tijcore::ModelPerceptionInterface::Ptr createModelPerceptionMixer() const;

  tijcore::Toolbox::SharedPtr createToolbox() const;

  std::vector<tijcore::ModelContainerInterface::Ptr>
  createModelContainers(const tijcore::Toolbox::SharedPtr &toolbox) const;

  std::vector<tijros::ConveyorBeltSurfaceFrameBroadcaster::Ptr>
  createConveyorBeltTransformBroadcasters() const;

  std::vector<tijcore::PickAndPlaceRobotInterface::Ptr>
  createPickAndPlaceRobots(const tijcore::Toolbox::SharedPtr &toolbox) const;
};

} // namespace tijchallenger
