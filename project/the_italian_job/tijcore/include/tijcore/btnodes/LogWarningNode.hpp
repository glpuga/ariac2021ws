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
class LogWarningNode : public BT::SyncActionNode
{
public:
  LogWarningNode(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("message") };
  }

  BT::NodeStatus tick() override
  {
    const auto message = getInput<std::string>("message").value();
    WARNING(message);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace tijcore
