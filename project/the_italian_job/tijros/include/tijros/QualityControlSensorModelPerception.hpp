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
class QualityControlSensorModelPerception : public tijcore::ModelPerceptionInterface
{
public:
  QualityControlSensorModelPerception(const ros::NodeHandle& nh,
                                      const std::string& quality_sensor_name);

  std::vector<tijcore::ObservedModel> getObservedModels() const override;

private:
  mutable std::mutex mutex_;

  ros::NodeHandle nh_;
  ros::Subscriber camera_sub_;

  bool update_received_{ false };
  std::string quality_sensor_name_;

  std::vector<tijcore::ObservedModel> models_;

  tijutils::Timer timer_;

  void cameraCallback(nist_gear::LogicalCameraImage::ConstPtr msg);

  void timerCallback();
};

}  // namespace tijros
