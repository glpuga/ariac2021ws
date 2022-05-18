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
class GetRobotInSafePoseNearTargetNode : public BT::SyncActionNode
{
public:
  GetRobotInSafePoseNearTargetNode(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<BTTaskParameters::SharedPtr>("task_parameters"),
      BT::InputPort<tijmath::RelativePose3>("target_pose"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto target_pose = getInput<tijmath::RelativePose3>("target_pose").value();
    auto task_parameters = getInput<BTTaskParameters::SharedPtr>("task_parameters").value();
    const auto adapter_ = task_parameters->primary_robot.value().resource();
    const auto retval = adapter_->getRobotInSafePoseNearTarget(target_pose);
    return retval ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace tijcore
