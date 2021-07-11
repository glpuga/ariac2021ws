/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>

// roscpp
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

// tijcore
#include <tijcore/perception/FrameTransformerInterface.hpp>

namespace tijros {

using tijcore::RelativePose3;

class ROSFrameTransformer : public tijcore::FrameTransformerInterface {
public:
  ROSFrameTransformer();

  RelativePose3
  transformPoseToFrame(const RelativePose3 &pose,
                       const std::string &new_frame_id) const override;

private:
  const ros::Duration tf_timeout_{1.5};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
};

} // namespace tijros
