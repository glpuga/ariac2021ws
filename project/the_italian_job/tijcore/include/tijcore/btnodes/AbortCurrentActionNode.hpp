/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <string>
#include <utility>

// external
#include <tijcore/abstractions/PickAndPlaceRobotMovementsInterface.hpp>

// tijcore
#include "behaviortree_cpp_v3/action_node.h"

namespace tijcore
{
class AbortCurrentActionNode : public BT::SyncActionNode
{
public:
  AbortCurrentActionNode(const std::string& name, const BT::NodeConfiguration& config,
                         PickAndPlaceRobotMovementsInterface::Ptr adapter)
    : SyncActionNode(name, config), adapter_{ std::move(adapter) }
  {
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    adapter_->abortCurrentAction();
    return BT::NodeStatus::SUCCESS;
  }

private:
  PickAndPlaceRobotMovementsInterface::Ptr adapter_;
};

}  // namespace tijcore