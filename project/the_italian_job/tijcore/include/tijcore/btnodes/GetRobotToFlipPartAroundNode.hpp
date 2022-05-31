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
class GetRobotToFlipPartAroundNode : public BT::AsyncActionNode
{
public:
  GetRobotToFlipPartAroundNode(const std::string& name, const BT::NodeConfiguration& config)
    : AsyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<BTTaskParameters::SharedPtr>("task_parameters"),
      BT::BidirectionalPort<tijmath::RelativePose3>("target_pose"),
      BT::InputPort<double>("offset_to_top"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto task_parameters = getInput<BTTaskParameters::SharedPtr>("task_parameters").value();
    const auto adapter_ = task_parameters->primary_robot.value().resource();

    auto target_pose = getInput<tijmath::RelativePose3>("target_pose").value();
    auto offset_to_top = getInput<double>("offset_to_top").value();

    const auto retval = adapter_->twistPartInPlace(target_pose, offset_to_top);

    setOutput("target_pose", target_pose);

    return retval ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  void halt() override
  {
  }
};

}  // namespace tijcore
