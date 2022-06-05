/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <deque>
#include <mutex>
#include <optional>
#include <string>

// external
#include <nist_gear/Proximity.h>

// ros
#include <ros/ros.h>

// project
#include <tijcore/abstractions/ConveyorBeltManagerInterface.hpp>
#include <tijmath/RelativePose3.hpp>

namespace tijros
{
class ConveyorBeltManager : public tijcore::ConveyorBeltManagerInterface
{
public:
  ConveyorBeltManager(const ros::NodeHandle& nh, const std::string& belt_break_beam_topic,
                      const tijmath::RelativePose3& detection_pose);

  std::optional<DetectionData> popNextDetectionTimestamp() override;

  tijmath::RelativePose3
  currentPositionInTheBelt(const DetectionData& detection_data) const override;

private:
  ros::NodeHandle nh_;
  mutable std::mutex mutex_;

  tijmath::RelativePose3 detection_pose_;

  ros::Subscriber belt_break_beam_sub_;

  std::deque<DetectionData> detections_;

  bool latest_detection_state_{ false };

  void beltBreakbeamCallback(nist_gear::Proximity::ConstPtr msg);
};

}  // namespace tijros
