/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <string>
#include <utility>

// external
#include "behaviortree_cpp_v3/action_node.h"

// tijcore
#include <tijcore/tasking/BTTaskParameters.hpp>

namespace tijcore
{
class SwapSrcAndAuxiliarLociNode : public BT::SyncActionNode
{
public:
  SwapSrcAndAuxiliarLociNode(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<BTTaskParameters::SharedPtr>("task_parameters"),
    };
  }

  BT::NodeStatus tick() override
  {
    BTTaskParameters::SharedPtr task_parameters =
        getInput<BTTaskParameters::SharedPtr>("task_parameters").value();
    std::swap(task_parameters->src_locus, task_parameters->aux_locus);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace tijcore
