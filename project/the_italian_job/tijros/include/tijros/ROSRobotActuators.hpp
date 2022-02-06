/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

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
#include <tijcore/competition/RobotActuatorsInterface.hpp>

namespace tijros
{
class ROSRobotActuators : public tijcore::RobotActuatorsInterface
{
public:
  explicit ROSRobotActuators(const ros::NodeHandle nh);

  ConveyorState getConveyorState() const override;

  VacuumGripperState getGantryGripperState() const override;

  bool setGantryGripperSuction(const bool enable) const override;

  VacuumGripperState getKittingGripperState() const override;

  bool setKittingGripperSuction(const bool enable) const override;

  bool setGantryTrayLockState(const bool lock_state) const override;

  RobotHealthStatus getRobotHealthStatus() const override;

private:
  mutable std::mutex mutex_;

  ros::NodeHandle nh_;

  ros::Subscriber conveyor_state_sub_;
  ros::Subscriber gantry_arm_gripper_state_sub_;
  ros::Subscriber kitting_arm_gripper_state_sub_;
  ros::Subscriber robot_health_sub_;

  ConveyorState latest_conveyor_state_data_;
  VacuumGripperState latest_gantry_arm_gripper_state_data_;
  VacuumGripperState latest_kitting_arm_gripper_state_data_;
  RobotHealthStatus latest_robot_health_data_;

  void conveyorStateCallback(nist_gear::ConveyorBeltState::ConstPtr msg);

  void gantryArmGripperStateCallback(nist_gear::VacuumGripperState::ConstPtr msg);

  void kittingArmGripperStateCallback(nist_gear::VacuumGripperState::ConstPtr msg);

  void robotHealthCallback(nist_gear::RobotHealth::ConstPtr msg);
};

}  // namespace tijros
