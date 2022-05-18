/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <string>

// external
#include "behaviortree_cpp_v3/action_node.h"

// tijcore
#include <tijcore/btnodes/behavior_tree_data_conversions.hpp>
#include <tijcore/datatypes/GripperTypeId.hpp>
#include <tijcore/tasking/BTTaskParameters.hpp>

namespace tijcore
{
class ToolTypesAreTheSameNode : public BT::SyncActionNode
{
public:
  ToolTypesAreTheSameNode(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<GripperTypeId>("left_hand_side"),
      BT::InputPort<GripperTypeId>("right_hand_side"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto left_hand_side = getInput<GripperTypeId>("left_hand_side").value();
    const auto right_hand_side = getInput<GripperTypeId>("right_hand_side").value();
    return left_hand_side == right_hand_side ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace tijcore
