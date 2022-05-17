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
class CalculateVerticalDropPoseNode : public BT::SyncActionNode
{
public:
  CalculateVerticalDropPoseNode(const std::string& name, const BT::NodeConfiguration& config,
                                PickAndPlaceRobotMovementsInterface::Ptr adapter)
    : SyncActionNode(name, config), adapter_{ std::move(adapter) }
  {
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<tijmath::RelativePose3>("target_pose"),
      BT::InputPort<tijmath::RelativePose3>("offset_to_top"),
      BT::OutputPort<tijmath::RelativePose3>("end_effector_pose"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto target_pose = getInput<tijmath::RelativePose3>("target_pose").value();
    auto offset_to_top = getInput<double>("offset_to_top").value();
    const auto end_effector_pose = adapter_->calculateVerticalDropPose(target_pose, offset_to_top);
    setOutput("end_effector_pose", end_effector_pose);
    return BT::NodeStatus::SUCCESS;
  }

private:
  PickAndPlaceRobotMovementsInterface::Ptr adapter_;
};

}  // namespace tijcore
