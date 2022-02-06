/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// standard library
#include <string>

// ros
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// tijcore
#include <logger/logger.hpp>
#include <tijros/ROSFrameTransformer.hpp>
#include <tijros/utils/utils.hpp>

namespace tijros
{
ROSFrameTransformer::ROSFrameTransformer()
{
}

RelativePose3 ROSFrameTransformer::transformPoseToFrame(const RelativePose3& relative_pose,
                                                        const std::string& new_frame_id) const
{
  // TODO(glpuga) We should be able to use Pose instead of PoseStamped here.
  geometry_msgs::PoseStamped ros_input_pose;

  ros_input_pose.header.stamp = ros::Time::now();
  ros_input_pose.header.frame_id = relative_pose.frameId();
  ros_input_pose.pose = utils::convertCorePoseToGeoPose(relative_pose.pose());

  // TODO(glpuga) We should be able to use Pose instead of PoseStamped here.
  geometry_msgs::PoseStamped ros_output_pose;

  try
  {
    auto start = ros::Time::now();
    tf2::doTransform(ros_input_pose, ros_output_pose,
                     tf_buffer_.lookupTransform(new_frame_id, relative_pose.frameId(), ros::Time(0), tf_timeout_));
    auto end = ros::Time::now();
    if (end - start > ros::Duration(0.1))
    {
      WARNING("A tf2 transform took {} seconds to complete!", end - start);
      WARNING("Details: {} to {}", relative_pose, new_frame_id);
    }
  }
  catch (const std::exception& ex)
  {
    ERROR("Error while tranforming between frames {} and {}: {}", relative_pose.frameId(), new_frame_id, ex.what());
    throw std::runtime_error{ "Failed ROS frame transformation" };
  }

  auto output_frame_id = ros_output_pose.header.frame_id;
  auto local_output_pose = utils::convertGeoPoseToCorePose(ros_output_pose.pose);
  tijcore::RelativePose3 output_pose{ output_frame_id, local_output_pose };

  DEBUG("Transformed pose {} to frame {}, resulting in the pose {}", relative_pose, new_frame_id, output_pose);

  return output_pose;
}

}  // namespace tijros
