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
class TestIfRobotReachesPoseNode : public BT::SyncActionNode
{
public:
  TestIfRobotReachesPoseNode(const std::string& name, const BT::NodeConfiguration& config,
                             PickAndPlaceRobotMovementsInterface::Ptr adapter)
    : SyncActionNode(name, config), adapter_{ std::move(adapter) }
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<tijmath::RelativePose3>("target_pose"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto target_pose = getInput<tijmath::RelativePose3>("target_pose").value();
    const auto does_reach = adapter_->testIfRobotReachesPose(target_pose);
    return does_reach ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  PickAndPlaceRobotMovementsInterface::Ptr adapter_;
};

}  // namespace tijcore
