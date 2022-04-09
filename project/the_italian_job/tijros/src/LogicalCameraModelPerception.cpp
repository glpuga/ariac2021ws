/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <algorithm>
#include <chrono>
#include <set>
#include <string>
#include <vector>

// ros
#include <ros/ros.h>

// tijcore
#include <tijlogger/logger.hpp>
#include <tijros/LogicalCameraModelPerception.hpp>
#include <tijros/utils/utils.hpp>

namespace tijros
{
namespace
{
constexpr char topic_prefix_[] = "/ariac/";
constexpr int default_queue_len_ = 10;

constexpr std::chrono::seconds timer_interval_{ 1 };

}  // namespace

LogicalCameraModelPerception::LogicalCameraModelPerception(const ros::NodeHandle& nh,
                                                           const std::string& logical_sensor_name)
  : nh_{ nh }, logical_sensor_name_{ logical_sensor_name }, timer_{ [this] { timerCallback(); } }
{
  const std::string topic_id{ topic_prefix_ + logical_sensor_name_ };
  camera_sub_ = nh_.subscribe(topic_id, default_queue_len_,
                              &LogicalCameraModelPerception::cameraCallback, this);
  timer_.start(timer_interval_);
}

std::vector<tijcore::ObservedItem> LogicalCameraModelPerception::getObservedModels() const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  return models_;
}

void LogicalCameraModelPerception::cameraCallback(nist_gear::LogicalCameraImage::ConstPtr msg)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  update_received_ = true;
  models_.clear();
  for (const auto& ros_model : msg->models)
  {
    const auto& geo_pose = ros_model.pose;
    const auto relative_core_pose =
        tijmath::RelativePose3{ logical_sensor_name_ + "_frame",
                                utils::convertGeoPoseToCorePose(geo_pose) };
    const tijcore::PartId part_id{ ros_model.type };
    const tijcore::ObservedItem core_model{ part_id, relative_core_pose, false };
    models_.emplace_back(core_model);
  }
}

void LogicalCameraModelPerception::timerCallback()
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  if (!update_received_)
  {
    WARNING("No updates on {}, possibly going through a sensor blackout", logical_sensor_name_);
    models_.clear();
  }
  update_received_ = false;
}

}  // namespace tijros
