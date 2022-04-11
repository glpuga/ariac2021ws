/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

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
  LogicalCameraModelPerception(const ros::NodeHandle& nh, const std::string& logical_sensor_name,
                               const ros::Duration& retention_interval);

  std::vector<tijcore::ObservedItem> getObservedModels() const override;

private:
  mutable std::mutex mutex_;

  std::string logical_sensor_name_;
  ros::Duration retention_interval_;

  ros::NodeHandle nh_;
  ros::Subscriber camera_sub_;

  std::vector<tijcore::ObservedItem> models_;
  ros::Time latest_update_timestamp_{ 0 };

  void cameraCallback(nist_gear::LogicalCameraImage::ConstPtr msg);
};

}  // namespace tijros
