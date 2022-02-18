/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <mutex>
#include <string>
#include <vector>

// ros
#include <nist_gear/LogicalCameraImage.h>
#include <ros/ros.h>

// tijcore
#include <tijcore/abstractions/ModelPerceptionInterface.hpp>
#include <tijutils/Timer.hpp>

namespace tijros
{
class LogicalCameraModelPerception : public tijcore::ModelPerceptionInterface
{
public:
  LogicalCameraModelPerception(const ros::NodeHandle& nh, const std::string& logical_sensor_name);

  std::vector<tijcore::ObservedModel> getObservedModels() const override;

private:
  mutable std::mutex mutex_;

  ros::NodeHandle nh_;
  ros::Subscriber camera_sub_;

  bool update_received_{ false };
  std::string logical_sensor_name_;

  tijutils::Timer timer_;

  std::vector<tijcore::ObservedModel> models_;

  void cameraCallback(nist_gear::LogicalCameraImage::ConstPtr msg);

  void timerCallback();
};

}  // namespace tijros
