/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <algorithm>
#include <string>

// external
#include "behaviortree_cpp_v3/action_node.h"

// tijcore
#include <tijcore/abstractions/ResourceManagerInterface.hpp>
#include <tijcore/tasking/BTTaskParameters.hpp>

namespace tijcore
{
class HackyDestinationPoseUpdateNode : public BT::SyncActionNode
{
public:
  HackyDestinationPoseUpdateNode(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<BTTaskParameters::SharedPtr>("task_parameters"),
      BT::InputPort<tijmath::RelativePose3>("new_destination_pose"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto task_parameters = getInput<BTTaskParameters::SharedPtr>("task_parameters").value();
    const auto new_destination_pose =
        getInput<tijmath::RelativePose3>("new_destination_pose").value();

    task_parameters->src_locus->resource()->pose() = new_destination_pose;

    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace tijcore
