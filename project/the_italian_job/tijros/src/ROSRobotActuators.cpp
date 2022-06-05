/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// standard library
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// roscpp
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

// competition
#include <nist_gear/ChangeGripper.h>
#include <nist_gear/ConveyorBeltState.h>
#include <nist_gear/RobotHealth.h>
#include <nist_gear/VacuumGripperControl.h>
#include <nist_gear/VacuumGripperState.h>
#include <std_msgs/String.h>

// tijcore
#include <tijcore/datatypes/GripperTypeId.hpp>
#include <tijlogger/logger.hpp>
#include <tijros/ROSRobotActuators.hpp>
#include <tijros/ROSRobotJointDirectControl.hpp>
namespace tijros
{
using tijcore::RobotActuatorsInterface;

namespace
{
constexpr int short_latency_queue_subscriber_len = 1;

constexpr char converyor_state_topic[] = "/ariac/conveyor/state";

constexpr char gantry_arm_gripper_state_topic[] = "/ariac/gantry/arm/gripper/state";
constexpr char kitting_arm_gripper_state_topic[] = "/ariac/kitting/arm/gripper/state";

constexpr char gantry_arm_gripper_control_service[] = "/ariac/gantry/arm/gripper/control";
constexpr char kitting_arm_gripper_control_service[] = "/ariac/kitting/arm/gripper/control";

constexpr char gantry_tray_lock_models_service[] = "/ariac/gantry_tray/lock_models";
constexpr char gantry_tray_unlock_models_service[] = "/ariac/gantry_tray/unlock_models";

constexpr char gantry_arm_gripper_tool_change_service[] = "/ariac/gantry/arm/gripper/change";

constexpr char robot_health_topic[] = "/ariac/robot_health";

constexpr char gripper_type_topic[] = "/ariac/gantry/arm/gripper/type";

};  // namespace

ROSRobotActuators::ROSRobotActuators(const ros::NodeHandle nh) : nh_{ nh }
{
  conveyor_state_sub_ = nh_.subscribe(converyor_state_topic, short_latency_queue_subscriber_len,
                                      &ROSRobotActuators::conveyorStateCallback, this);

  gantry_arm_gripper_state_sub_ =
      nh_.subscribe(gantry_arm_gripper_state_topic, short_latency_queue_subscriber_len,
                    &ROSRobotActuators::gantryArmGripperStateCallback, this);

  kitting_arm_gripper_state_sub_ =
      nh_.subscribe(kitting_arm_gripper_state_topic, short_latency_queue_subscriber_len,
                    &ROSRobotActuators::kittingArmGripperStateCallback, this);

  robot_health_sub_ = nh_.subscribe(robot_health_topic, short_latency_queue_subscriber_len,
                                    &ROSRobotActuators::robotHealthCallback, this);

  gripper_type_sub_ = nh_.subscribe(gripper_type_topic, short_latency_queue_subscriber_len,
                                    &ROSRobotActuators::gripperTypeCallback, this);

  kitting_joint_control_ = std::make_unique<ROSRobotJointDirectControl>(
      nh_, "/ariac/kitting/joint_states", "/ariac/kitting/kitting_arm_controller/command",
      std::vector<std::string>{ "linear_arm_actuator_joint", "shoulder_pan_joint",
                                "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
                                "wrist_2_joint", "wrist_3_joint" },
      std::vector<double>{ 1.0, 2.16, 2.16, 3.15, 3.2, 3.2, 3.2 });
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

bool ROSRobotActuators::setGantryGripperToolType(const tijcore::GripperTypeId new_type) const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  nist_gear::ChangeGripper msg;
  msg.request.gripper_type = tijcore::gripper_type::toString(new_type);
  ros::service::call(gantry_arm_gripper_tool_change_service, msg);
  if (!msg.response.success)
  {
    ERROR("Failed to set a new gantry gripper tool to {}, error msg: {}", new_type,
          msg.response.message);
    return false;
  }
  INFO("Updated gantry gripper tool to:", new_type);
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

tijcore::GripperTypeId ROSRobotActuators::getGantryGripperToolType() const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  return latest_gripper_type_data_;
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

  auto str_to_bool = [](const std::string& str_value) { return str_value == "active"; };

  latest_robot_health_data_.kitting_robot_enabled = str_to_bool(msg->kitting_robot_health);
  latest_robot_health_data_.assembly_robot_enabled = str_to_bool(msg->assembly_robot_health);
}

void ROSRobotActuators::gripperTypeCallback(std_msgs::String::ConstPtr msg)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  if (tijcore::gripper_type::isValid(msg->data))
  {
    const auto new_gripper_type_value = tijcore::gripper_type::fromString(msg->data);
    if (new_gripper_type_value != latest_gripper_type_data_)
    {
      latest_gripper_type_data_ = new_gripper_type_value;
      INFO("Updated gripper type to {}", new_gripper_type_value);
    }
  }
  else
  {
    ERROR("Received invalid gripper type: {}", msg->data);
  }
}

tijcore::RobotJointDirectControlInterface& ROSRobotActuators::getKittingJointDirectControlManager()
{
  return *kitting_joint_control_;
}

tijcore::RobotJointDirectControlInterface& ROSRobotActuators::getGantryJointDirectControlManager()
{
  throw std::runtime_error{ "Not implemented" };
  return *gantry_joint_control_;
}

}  // namespace tijros
