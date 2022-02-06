/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library
#include <chrono>
#include <string>
#include <vector>

// ros
#include <ros/ros.h>

// tijcore
#include <tijcore/logger/logger.hpp>
#include <tijros/QualityControlSensorModelPerception.hpp>
#include <tijros/utils/utils.hpp>

namespace tijros
{
namespace
{
constexpr char topic_prefix[] = "/ariac/";
constexpr int default_queue_len = 10;

constexpr std::chrono::seconds timer_interval_{ 1 };
}  // namespace

QualityControlSensorModelPerception::QualityControlSensorModelPerception(const ros::NodeHandle& nh,
                                                                         const std::string& quality_sensor_name)
  : nh_{ nh }, quality_sensor_name_{ quality_sensor_name }, timer_{ [this] { timerCallback(); } }
{
  const std::string topic_id{ topic_prefix + quality_sensor_name_ };
  camera_sub_ = nh_.subscribe(topic_id, default_queue_len, &QualityControlSensorModelPerception::cameraCallback, this);
  timer_.start(timer_interval_);
}

std::vector<tijcore::ObservedModel> QualityControlSensorModelPerception::getObservedModels() const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  return models_;
}

void QualityControlSensorModelPerception::cameraCallback(nist_gear::LogicalCameraImage::ConstPtr msg)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  update_received_ = true;
  models_.clear();
  for (const auto& ros_model : msg->models)
  {
    const auto& geo_pose = ros_model.pose;
    const auto relative_core_pose =
        tijcore::RelativePose3{ quality_sensor_name_ + "_frame", utils::convertGeoPoseToCorePose(geo_pose) };
    const auto part_id = tijcore::PartId::UnkownPartId;
    // quality sensor only report faulty parts
    const tijcore::ObservedModel core_model{ part_id, relative_core_pose, true };
    models_.emplace_back(core_model);
  }
}

void QualityControlSensorModelPerception::timerCallback()
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  if (!update_received_)
  {
    WARNING("No updates on {}, possibly going through a sensor blackout", quality_sensor_name_);
    models_.clear();
  }
  update_received_ = false;
}

}  // namespace tijros
