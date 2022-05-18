/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <string>

// external
#include "behaviortree_cpp_v3/action_node.h"

// tijcore
#include <tijcore/tasking/BTTaskData.hpp>

namespace tijcore
{
class GetRobotArmInRestingPoseNode : public BT::AsyncActionNode
{
public:
  GetRobotArmInRestingPoseNode(const std::string& name, const BT::NodeConfiguration& config)
    : AsyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<BTTaskData::SharedPtr>("task_parameters"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto task_parameters = getInput<BTTaskData::SharedPtr>("task_parameters").value();
    const auto adapter_ = task_parameters->primary_robot.value().resource();
    const auto retval = adapter_->getRobotArmInRestingPose();
    return retval ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  void halt() override
  {
    auto task_parameters = getInput<BTTaskData::SharedPtr>("task_parameters").value();
    const auto adapter_ = task_parameters->primary_robot.value().resource();
    adapter_->abortCurrentAction();
  }
};

}  // namespace tijcore
