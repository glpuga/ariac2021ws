/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>
#include <string>

// ros
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace tijros {

class ConveyorBeltSurfaceFrameBroadcaster {
public:
  using Ptr = std::unique_ptr<ConveyorBeltSurfaceFrameBroadcaster>;

  ConveyorBeltSurfaceFrameBroadcaster(
      const ros::NodeHandle &nh,
      const std::string &container_reference_frame_id,
      const std::string &surface_reference_frame_id,
      const double conveyor_belt_speed_meters_per_second,
      const double publication_rate = 20.0)
      : nh_{nh}, container_reference_frame_id_{container_reference_frame_id},
        surface_reference_frame_id_{surface_reference_frame_id},
        conveyor_belt_speed_meters_per_second_{
            conveyor_belt_speed_meters_per_second},
        start_time_{ros::Time::now()} {
    updateAndPublishTransform();
    auto callback = [this](const ros::TimerEvent &) {
      updateAndPublishTransform();
    };
    timer_ = nh_.createTimer(ros::Duration(1.0 / publication_rate), callback);
  }

private:
  ros::NodeHandle nh_;
  std::string container_reference_frame_id_;
  std::string surface_reference_frame_id_;
  double conveyor_belt_speed_meters_per_second_;
  ros::Time start_time_;
  ros::Timer timer_;

  tf::TransformBroadcaster broadcaster_;

  void updateAndPublishTransform() {
    const auto now = ros::Time::now();
    const auto conveyor_belt_offset =
        -(now - start_time_).toSec() * conveyor_belt_speed_meters_per_second_;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0, conveyor_belt_offset, 0.0));
    transform.setRotation(tf::Quaternion{0, 0, 0, 1});
    broadcaster_.sendTransform(
        tf::StampedTransform(transform, now, container_reference_frame_id_,
                             surface_reference_frame_id_));
  }
};

}; // namespace tijros