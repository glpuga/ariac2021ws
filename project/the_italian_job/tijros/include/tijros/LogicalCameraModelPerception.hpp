/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <mutex>
#include <vector>

// ros
#include <nist_gear/LogicalCameraImage.h>
#include <ros/ros.h>

// tijcore
#include <tijcore/perception/ModelPerceptionInterface.hpp>
#include <tijcore/utils/Timer.hpp>

namespace tijros {

class LogicalCameraModelPerception : public tijcore::ModelPerceptionInterface {
public:
  LogicalCameraModelPerception(const ros::NodeHandle &nh,
                               const std::string &logical_sensor_name);

  std::vector<tijcore::ObservedModel> getObservedModels() const override;

private:
  mutable std::mutex mutex_;

  ros::NodeHandle nh_;
  ros::Subscriber camera_sub_;

  bool update_received_{false};
  std::string logical_sensor_name_;

  tijcore::utils::Timer timer_;

  std::vector<tijcore::ObservedModel> models_;

  void cameraCallback(nist_gear::LogicalCameraImage::ConstPtr msg);

  void timerCallback();
};

} // namespace tijros
