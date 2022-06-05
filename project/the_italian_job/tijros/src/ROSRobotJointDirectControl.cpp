/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <algorithm>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <vector>

// ros
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

// tijcore
#include <tijlogger/logger.hpp>
#include <tijros/ROSRobotJointDirectControl.hpp>

namespace tijros
{
ROSRobotJointDirectControl::ROSRobotJointDirectControl(const ros::NodeHandle nh,
                                                       const std::string& joint_status_topic,
                                                       const std::string& joint_control_topic,
                                                       const std::vector<std::string>& joint_names,
                                                       const std::vector<double>& velocity_limits)
  : nh_{ nh }, joint_names_{ joint_names }
{
  // ensure that we can access the map without checking if the elements are there
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    desired_joint_values_.insert({ joint_names[i], 0.0 });
    velocity_limits_.insert({ joint_names[i], velocity_limits[i] });
  }

  joint_control_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(joint_control_topic, 10);
  joint_status_sub_ =
      nh_.subscribe(joint_status_topic, 10, &ROSRobotJointDirectControl::jointStatusCallback, this);
}

bool ROSRobotJointDirectControl::setJointPosition(
    const std::map<std::string, double>& updated_values, const double interval)
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (const auto& item : updated_values)
  {
    const auto& joint_name = item.first;
    const auto& value = item.second;

    const auto max_delta = std::abs(velocity_limits_[joint_name] * interval);

    // bound the value with the speed limits
    const auto delta = value - desired_joint_values_[joint_name];
    const auto limited_delta = std::max(-max_delta, std::min(max_delta, delta));

    desired_joint_values_[joint_name] += limited_delta;
  }

  {
    trajectory_msgs::JointTrajectory msg;
    msg.header.stamp = ros::Time::now();
    msg.joint_names = joint_names_;
    msg.points.resize(1);
    msg.points[0].positions.reserve(joint_names_.size());

    for (const auto& joint_name : joint_names_)
    {
      double value = 0.0;
      if (desired_joint_values_.find(joint_name) != desired_joint_values_.end())
      {
        value = desired_joint_values_[joint_name];
      }
      msg.points[0].positions.push_back(value);
    }
    msg.points[0].time_from_start = ros::Duration(interval);

    joint_control_pub_.publish(msg);
  }

  return true;
}

bool ROSRobotJointDirectControl::inTrackingMode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return in_tracking_mode_;
}

void ROSRobotJointDirectControl::jointStatusCallback(sensor_msgs::JointState::ConstPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if ((msg->position.size() != desired_joint_values_.size()) ||
      (msg->name.size() != desired_joint_values_.size()))
  {
    ERROR("Joint status vector lenght does not match intended settings: {}", msg->position.size())
    in_tracking_mode_ = false;
  }

  in_tracking_mode_ = true;
  for (size_t i = 0; i < msg->position.size(); ++i)
  {
    // there should not be a single threshold for both angular and linear joints
    const auto delta = std::abs(desired_joint_values_[msg->name[i]] - msg->position[i]);
    in_tracking_mode_ = (delta > 0.05) ? false : in_tracking_mode_;
  }
}

}  // namespace tijros
