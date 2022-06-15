/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <deque>
#include <mutex>
#include <string>

// external
#include <nist_gear/Proximity.h>

// ros
#include <ros/ros.h>

// tijcore
#include <tijlogger/logger.hpp>
#include <tijros/ConveyorBeltManager.hpp>

namespace tijros
{
namespace
{
constexpr std::size_t default_queue_len_ = 10;
constexpr double belt_speed_ = 0.40;
constexpr double too_old_threshold_ = 5.0 / belt_speed_;  // if it's pass mid belt, drop

}  // namespace

ConveyorBeltManager::ConveyorBeltManager(const ros::NodeHandle& nh,
                                         const std::string& belt_break_beam_topic,
                                         const tijmath::RelativePose3& detection_pose)
  : nh_{ nh }, detection_pose_{ detection_pose }
{
  belt_break_beam_sub_ = nh_.subscribe(belt_break_beam_topic, default_queue_len_,
                                       &ConveyorBeltManager::beltBreakbeamCallback, this);

  if (detection_pose.frameId() != "world")
  {
    throw std::runtime_error(
        "This is hacked together at the last minute, so the pose frame "
        "must be world.");
  }
}

void ConveyorBeltManager::beltBreakbeamCallback(nist_gear::Proximity::ConstPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (msg->object_detected != latest_detection_state_)
  {
    if (msg->object_detected)
    {
      DetectionData new_detection;
      new_detection.detection_timestamp = ros::Time::now().toSec();
      detections_.push_back(new_detection);
    }

    latest_detection_state_ = msg->object_detected;
  }
}

std::optional<ConveyorBeltManager::DetectionData> ConveyorBeltManager::popNextDetectionTimestamp()
{
  std::lock_guard<std::mutex> lock(mutex_);
  while (!detections_.empty())
  {
    const auto detection_data = detections_.front();
    detections_.pop_front();

    if (detection_data.detection_timestamp > too_old_threshold_)
    {
      continue;
    }

    return detection_data;
  }
  return std::nullopt;
}

tijmath::RelativePose3
ConveyorBeltManager::currentPositionInTheBelt(const DetectionData& detection_data) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto seconds_since_detection =
      ros::Time::now().toSec() - detection_data.detection_timestamp;

  auto pose = detection_pose_;
  pose.position().vector().y() -= seconds_since_detection * belt_speed_;

  return pose;
}

double ConveyorBeltManager::getBeltSpeed() const
{
  return belt_speed_;
}

}  // namespace tijros
