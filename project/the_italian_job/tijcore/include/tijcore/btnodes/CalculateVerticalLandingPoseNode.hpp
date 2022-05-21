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
class CalculateVerticalLandingPoseNode : public BT::SyncActionNode
{
public:
  CalculateVerticalLandingPoseNode(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<BTTaskParameters::SharedPtr>("task_parameters"),
      BT::InputPort<tijmath::RelativePose3>("target_pose"),
      BT::InputPort<double>("offset_to_top"),
      BT::OutputPort<tijmath::RelativePose3>("vertical_landing_pose"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto target_pose = getInput<tijmath::RelativePose3>("target_pose").value();
    auto offset_to_top = getInput<double>("offset_to_top").value();
    auto task_parameters = getInput<BTTaskParameters::SharedPtr>("task_parameters").value();
    const auto adapter_ = task_parameters->primary_robot.value().resource();
    const auto vertical_landing_pose =
        adapter_->calculateVerticalLandingPose(target_pose, offset_to_top);
    setOutput("vertical_landing_pose", vertical_landing_pose);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace tijcore
