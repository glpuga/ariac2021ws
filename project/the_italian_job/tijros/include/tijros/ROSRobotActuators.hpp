/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <mutex>

// roscpp
#include <ros/ros.h>

// competition
#include <nist_gear/ConveyorBeltState.h>
#include <nist_gear/RobotHealth.h>
#include <nist_gear/VacuumGripperState.h>

// tijcore
#include <tijcore/abstractions/RobotActuatorsInterface.hpp>
#include <tijcore/abstractions/RobotJointDirectControlInterface.hpp>

namespace tijros
{
class ROSRobotActuators : public tijcore::RobotActuatorsInterface
{
public:
  explicit ROSRobotActuators(const ros::NodeHandle nh);

  ConveyorState getConveyorState() const override;

  VacuumGripperState getGantryGripperState() const override;

  bool setGantryGripperSuction(const bool enable) const override;

  bool setGantryGripperToolType(const tijcore::GripperTypeId new_type) const override;

  VacuumGripperState getKittingGripperState() const override;

  bool setKittingGripperSuction(const bool enable) const override;

  bool setGantryTrayLockState(const bool lock_state) const override;

  RobotHealthStatus getRobotHealthStatus() const override;

  tijcore::GripperTypeId getGantryGripperToolType() const override;

  tijcore::RobotJointDirectControlInterface& getKittingJointDirectControlManager() override;

  tijcore::RobotJointDirectControlInterface& getGantryJointDirectControlManager() override;

private:
  mutable std::mutex mutex_;

  ros::NodeHandle nh_;

  ros::Subscriber conveyor_state_sub_;
  ros::Subscriber gantry_arm_gripper_state_sub_;
  ros::Subscriber kitting_arm_gripper_state_sub_;
  ros::Subscriber robot_health_sub_;
  ros::Subscriber gripper_type_sub_;

  tijcore::RobotJointDirectControlInterface::Ptr kitting_joint_control_;
  tijcore::RobotJointDirectControlInterface::Ptr gantry_joint_control_;

  ConveyorState latest_conveyor_state_data_;
  VacuumGripperState latest_gantry_arm_gripper_state_data_;
  VacuumGripperState latest_kitting_arm_gripper_state_data_;
  RobotHealthStatus latest_robot_health_data_;
  tijcore::GripperTypeId latest_gripper_type_data_;

  void conveyorStateCallback(nist_gear::ConveyorBeltState::ConstPtr msg);

  void gantryArmGripperStateCallback(nist_gear::VacuumGripperState::ConstPtr msg);

  void kittingArmGripperStateCallback(nist_gear::VacuumGripperState::ConstPtr msg);

  void robotHealthCallback(nist_gear::RobotHealth::ConstPtr msg);

  void gripperTypeCallback(std_msgs::String::ConstPtr msg);
};

}  // namespace tijros
