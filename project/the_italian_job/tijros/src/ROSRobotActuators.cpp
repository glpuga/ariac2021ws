/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// standard library
#include <mutex>

// roscpp
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

// competition
#include <nist_gear/ConveyorBeltState.h>
#include <nist_gear/RobotHealth.h>
#include <nist_gear/VacuumGripperControl.h>
#include <nist_gear/VacuumGripperState.h>

// tijcore
#include <tijlogger/logger.hpp>
#include <tijros/ROSRobotActuators.hpp>

namespace tijros
{
using tijcore::RobotActuatorsInterface;

namespace
{
constexpr int default_queue_len = 10;

constexpr char converyor_state_topic[] = "/ariac/conveyor/state";

constexpr char gantry_arm_gripper_state_topic[] = "/ariac/gantry/arm/gripper/state";
constexpr char kitting_arm_gripper_state_topic[] = "/ariac/kitting/arm/gripper/state";

constexpr char gantry_arm_gripper_control_service[] = "/ariac/gantry/arm/gripper/control";
constexpr char kitting_arm_gripper_control_service[] = "/ariac/kitting/arm/gripper/control";

constexpr char gantry_tray_lock_models_service[] = "/ariac/gantry_tray/lock_models";
constexpr char gantry_tray_unlock_models_service[] = "/ariac/gantry_tray/unlock_models";

constexpr char robot_health_topic[] = "/ariac/robot_health";

};  // namespace

ROSRobotActuators::ROSRobotActuators(const ros::NodeHandle nh) : nh_{ nh }
{
  conveyor_state_sub_ = nh_.subscribe(converyor_state_topic, default_queue_len,
                                      &ROSRobotActuators::conveyorStateCallback, this);

  gantry_arm_gripper_state_sub_ =
      nh_.subscribe(gantry_arm_gripper_state_topic, default_queue_len,
                    &ROSRobotActuators::gantryArmGripperStateCallback, this);

  kitting_arm_gripper_state_sub_ =
      nh_.subscribe(kitting_arm_gripper_state_topic, default_queue_len,
                    &ROSRobotActuators::kittingArmGripperStateCallback, this);

  robot_health_sub_ = nh_.subscribe(robot_health_topic, default_queue_len,
                                    &ROSRobotActuators::robotHealthCallback, this);
}

RobotActuatorsInterface::ConveyorState ROSRobotActuators::getConveyorState() const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  return latest_conveyor_state_data_;
}

RobotActuatorsInterface::VacuumGripperState ROSRobotActuators::getGantryGripperState() const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  return latest_gantry_arm_gripper_state_data_;
}

bool ROSRobotActuators::setGantryGripperSuction(const bool enable) const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  nist_gear::VacuumGripperControl msg;
  msg.request.enable = enable;
  ros::service::call(gantry_arm_gripper_control_service, msg);
  if (!msg.response.success)
  {
    ERROR("Failed to set a new gantry gripper state");
    return false;
  }
  INFO("Updated gantry gripper state:", enable);
  return true;
}

RobotActuatorsInterface::VacuumGripperState ROSRobotActuators::getKittingGripperState() const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  return latest_kitting_arm_gripper_state_data_;
}

bool ROSRobotActuators::setKittingGripperSuction(const bool enable) const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  nist_gear::VacuumGripperControl msg;
  msg.request.enable = enable;
  ros::service::call(kitting_arm_gripper_control_service, msg);
  if (!msg.response.success)
  {
    ERROR("Failed to set a newkitting gripper state");
    return false;
  }
  INFO("Updated gantry kitting state:", enable);
  return true;
}

bool ROSRobotActuators::setGantryTrayLockState(const bool lock_state) const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  std_srvs::Trigger msg;
  if (lock_state)
  {
    ros::service::call(gantry_tray_lock_models_service, msg);
  }
  else
  {
    ros::service::call(gantry_tray_unlock_models_service, msg);
  }
  if (!msg.response.success)
  {
    ERROR("Failed to set the gantry lock state to {}", lock_state ? "ON" : "OFF");
    return false;
  }
  INFO("Updated gantry tray lock state to {}", lock_state ? "ON" : "OFF");
  return true;
}

RobotActuatorsInterface::RobotHealthStatus ROSRobotActuators::getRobotHealthStatus() const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  return latest_robot_health_data_;
}

void ROSRobotActuators::conveyorStateCallback(nist_gear::ConveyorBeltState::ConstPtr msg)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  latest_conveyor_state_data_.power = msg->power;
  latest_conveyor_state_data_.enabled = msg->enabled;
}

void ROSRobotActuators::gantryArmGripperStateCallback(nist_gear::VacuumGripperState::ConstPtr msg)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  latest_gantry_arm_gripper_state_data_.enabled = msg->enabled;
  latest_gantry_arm_gripper_state_data_.attached = msg->attached;
}

void ROSRobotActuators::kittingArmGripperStateCallback(nist_gear::VacuumGripperState::ConstPtr msg)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  latest_kitting_arm_gripper_state_data_.enabled = msg->enabled;
  latest_kitting_arm_gripper_state_data_.attached = msg->attached;
}

void ROSRobotActuators::robotHealthCallback(nist_gear::RobotHealth::ConstPtr msg)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  latest_robot_health_data_.kitting_robot_enabled = msg->kitting_robot_enabled;
  latest_robot_health_data_.assembly_robot_enabled = msg->assembly_robot_enabled;
}

}  // namespace tijros
