/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <chrono>
#include <string>
#include <vector>

// ros
#include <ros/ros.h>

// tijcore
#include <tijlogger/logger.hpp>
#include <tijros/QualityControlSensorModelPerception.hpp>
#include <tijros/utils/utils.hpp>

namespace tijros
{
namespace
{
constexpr char topic_prefix[] = "/ariac/";
constexpr int default_queue_len = 10;

}  // namespace

QualityControlSensorModelPerception::QualityControlSensorModelPerception(
    const ros::NodeHandle& nh, const std::string& quality_sensor_name,
    const ros::Duration& retention_interval)
  : quality_sensor_name_{ quality_sensor_name }
  , retention_interval_{ retention_interval }
  , nh_{ nh }
{
  const std::string topic_id{ topic_prefix + quality_sensor_name_ };
  camera_sub_ = nh_.subscribe(topic_id, default_queue_len,
                              &QualityControlSensorModelPerception::cameraCallback, this);
}

std::vector<tijcore::ObservedItem> QualityControlSensorModelPerception::getObservedModels() const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  const auto now = ros::Time::now();
  if (now - latest_update_timestamp_ > retention_interval_)
  {
    return {};
  }
  return models_;
}

void QualityControlSensorModelPerception::cameraCallback(
    nist_gear::LogicalCameraImage::ConstPtr msg)
{
  std::lock_guard<std::mutex> lock{ mutex_ };

  latest_update_timestamp_ = ros::Time::now();
  const auto timestamp =
      std::chrono::system_clock::time_point(std::chrono::seconds{ latest_update_timestamp_.sec }) +
      std::chrono::nanoseconds{ latest_update_timestamp_.nsec };

  models_.clear();

  for (const auto& ros_model : msg->models)
  {
    const auto& geo_pose = ros_model.pose;
    const auto relative_core_pose =
        tijmath::RelativePose3{ quality_sensor_name_ + "_frame",
                                utils::convertGeoPoseToCorePose(geo_pose) };
    // quality sensor only report faulty parts
    const tijcore::ObservedItem core_model{ tijcore::QualifiedPartInfo{
                                                tijcore::PartId::UnkownPartId, true },
                                            relative_core_pose, timestamp };
    models_.emplace_back(core_model);
  }
}

}  // namespace tijros
