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
#include <tijcore/datatypes/MovableTrayId.hpp>
#include <tijcore/datatypes/PartId.hpp>
#include <tijcore/datatypes/QualifiedMovableTrayInfo.hpp>
#include <tijcore/datatypes/QualifiedPartInfo.hpp>
#include <tijlogger/logger.hpp>
#include <tijros/LogicalCameraModelPerception.hpp>
#include <tijros/utils/utils.hpp>

namespace tijros
{
namespace
{
constexpr char topic_prefix_[] = "/ariac/";
constexpr int default_queue_len_ = 10;

}  // namespace

LogicalCameraModelPerception::LogicalCameraModelPerception(const ros::NodeHandle& nh,
                                                           const std::string& logical_sensor_name,
                                                           const ros::Duration& retention_interval)
  : logical_sensor_name_{ logical_sensor_name }
  , retention_interval_{ retention_interval }
  , nh_{ nh }

{
  const std::string topic_id{ topic_prefix_ + logical_sensor_name_ };
  camera_sub_ = nh_.subscribe(topic_id, default_queue_len_,
                              &LogicalCameraModelPerception::cameraCallback, this);
}

std::vector<tijcore::ObservedItem> LogicalCameraModelPerception::getObservedModels() const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  const auto now = ros::Time::now();
  if (now - latest_update_timestamp_ > retention_interval_)
  {
    return {};
  }
  return models_;
}

void LogicalCameraModelPerception::cameraCallback(nist_gear::LogicalCameraImage::ConstPtr msg)
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
        tijmath::RelativePose3{ logical_sensor_name_ + "_frame",
                                utils::convertGeoPoseToCorePose(geo_pose) };

    if (tijcore::PartId::isValid(ros_model.type))
    {
      const tijcore::PartId part_id{ ros_model.type };
      const tijcore::ObservedItem core_model{ tijcore::QualifiedPartInfo{ part_id, false },
                                              relative_core_pose };
      models_.emplace_back(core_model);
    }
    else if (tijcore::movable_tray::isValid(ros_model.type))
    {
      const auto movable_tray_id = tijcore::movable_tray::fromString(ros_model.type);
      const tijcore::ObservedItem core_model{ tijcore::QualifiedMovableTrayInfo{ movable_tray_id },
                                              relative_core_pose, timestamp };
      models_.emplace_back(core_model);
    }
    else
    {
      ERROR("{} is not a valid part or movable tray id", ros_model.type);
    }
  }
}

}  // namespace tijros
