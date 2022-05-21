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
class GetSourceLocusRelativePoseNode : public BT::SyncActionNode
{
public:
  GetSourceLocusRelativePoseNode(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<BTTaskParameters::SharedPtr>("task_parameters"),
      BT::OutputPort<tijmath::RelativePose3>("source_locus_pose"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto task_parameters = getInput<BTTaskParameters::SharedPtr>("task_parameters").value();
    setOutput("source_locus_pose", task_parameters->src_locus.value().resource()->pose());
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace tijcore
