/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <map>
#include <mutex>
#include <string>
#include <vector>

// roscpp
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// tijcore
#include <tijcore/abstractions/RobotJointDirectControlInterface.hpp>

namespace tijros
{
class ROSRobotJointDirectControl : public tijcore::RobotJointDirectControlInterface
{
public:
  ROSRobotJointDirectControl(const ros::NodeHandle nh, const std::string& joint_status_topic,
                             const std::string& joint_control_topic,
                             const std::vector<std::string>& joint_names,
                             const std::vector<double>& velocity_limits);

  bool setJointPosition(const std::map<std::string, double>& updated_values,
                        const double interval) override;

  bool inTrackingMode() const override;

private:
  mutable std::mutex mutex_;

  ros::NodeHandle nh_;

  ros::Publisher joint_control_pub_;
  ros::Subscriber joint_status_sub_;

  std::vector<std::string> joint_names_;

  std::map<std::string, double> desired_joint_values_;
  std::map<std::string, double> velocity_limits_;

  bool in_tracking_mode_{ false };

  void jointStatusCallback(sensor_msgs::JointState::ConstPtr msg);
};

}  // namespace tijros
