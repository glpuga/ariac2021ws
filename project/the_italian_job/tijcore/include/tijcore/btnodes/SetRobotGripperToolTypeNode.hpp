/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <string>

// external
#include "behaviortree_cpp_v3/action_node.h"

// tijcore
#include <tijcore/datatypes/GripperTypeId.hpp>
#include <tijcore/tasking/BTTaskParameters.hpp>

namespace tijcore
{
class SetRobotGripperToolTypeNode : public BT::SyncActionNode
{
public:
  SetRobotGripperToolTypeNode(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<BTTaskParameters::SharedPtr>("task_parameters"),
      BT::InputPort<GripperTypeId>("gripper_tool_type"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto gripper_tool_type = getInput<GripperTypeId>("gripper_tool_type").value();
    auto task_parameters = getInput<BTTaskParameters::SharedPtr>("task_parameters").value();
    const auto adapter_ = task_parameters->primary_robot.value().resource();
    const auto success = adapter_->setRobotGripperToolType(gripper_tool_type);
    return success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace tijcore
