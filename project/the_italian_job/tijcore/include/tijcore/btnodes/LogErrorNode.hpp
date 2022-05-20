/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <string>

// external
#include "behaviortree_cpp_v3/action_node.h"

// tijcore
#include <tijlogger/logger.hpp>

namespace tijcore
{
class LogErrorNode : public BT::SyncActionNode
{
public:
  LogErrorNode(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("message") };
  }

  BT::NodeStatus tick() override
  {
    auto task_parameters = getInput<std::string>("message").value();
    ERROR(task_parameters);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace tijcore
