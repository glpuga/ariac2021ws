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
class HackyPartHeightCompensationNode : public BT::SyncActionNode
{
public:
  HackyPartHeightCompensationNode(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<BTTaskParameters::SharedPtr>("task_parameters"),
      BT::InputPort<double>("offset_to_top"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto task_parameters = getInput<BTTaskParameters::SharedPtr>("task_parameters").value();
    const auto offset_to_top = getInput<double>("offset_to_top").value();

    // hackish way to compensate for the difference in which heights are handled
    // for empty spaces (like destination) and parts (like target), because
    // cameras report part height at about mid-height, while empty spaces
    // have locations on the surface.
    task_parameters->dst_locus->resource()->pose().position().vector().z() += offset_to_top;
    // the former also assumes offset_to_top == offset_to_bottom

    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace tijcore
