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
class LockMovableTrayInAGVNode : public BT::AsyncActionNode
{
public:
  LockMovableTrayInAGVNode(const std::string& name, const BT::NodeConfiguration& config)
    : AsyncActionNode(name, config)
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
    auto task_parameters = getInput<BTTaskParameters::SharedPtr>("task_parameters").value();
    auto toolbox = task_parameters->toolbox;
    auto proces_manager = toolbox->getProcessManager();

    INFO("Locking movable tray on AGV {}", task_parameters->agv_id);
    proces_manager->lockTrayInAgv(task_parameters->agv_id);

    return BT::NodeStatus::SUCCESS;
  }

  void halt() override
  {
  }
};

}  // namespace tijcore
