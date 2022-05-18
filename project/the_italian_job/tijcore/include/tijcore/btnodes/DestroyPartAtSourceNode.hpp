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
class DestroyPartAtSourceNode : public BT::SyncActionNode
{
public:
  DestroyPartAtSourceNode(const std::string& name, const BT::NodeConfiguration& config)
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
    auto transient_limbo = ManagedLocus::CreateEmptyLocus("drop_bucket", {});
    ManagedLocus::TransferPartFromHereToThere(*task_parameters->src_locus.value().resource(),
                                              transient_limbo);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace tijcore
