/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <string>

// external
#include "behaviortree_cpp_v3/action_node.h"

// tijcore
#include <tijcore/tasking/BTTaskParameters.hpp>

namespace tijcore
{
class SetRobotGripperStateNode : public BT::SyncActionNode
{
public:
  SetRobotGripperStateNode(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<BTTaskParameters::SharedPtr>("task_parameters"),
      BT::InputPort<bool>("gripper_state"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto gripper_state = getInput<bool>("gripper_state").value();
    auto task_parameters = getInput<BTTaskParameters::SharedPtr>("task_parameters").value();
    const auto adapter_ = task_parameters->primary_robot.value().resource();
    adapter_->setRobotGripperState(gripper_state);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace tijcore
