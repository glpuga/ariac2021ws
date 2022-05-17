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
class CalculateEndEffectorToPayloadTransformNode : public BT::SyncActionNode
{
public:
  CalculateEndEffectorToPayloadTransformNode(const std::string& name,
                                             const BT::NodeConfiguration& config,
                                             PickAndPlaceRobotMovementsInterface::Ptr adapter)
    : SyncActionNode(name, config), adapter_{ std::move(adapter) }
  {
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<tijmath::RelativePose3>("end_effector_pose"),
      BT::InputPort<tijmath::RelativePose3>("payload_pose"),
      BT::OutputPort<tijmath::Pose3>("end_effector_to_payload_transform"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto end_effector_pose = getInput<tijmath::RelativePose3>("end_effector_pose").value();
    auto payload_pose = getInput<tijmath::RelativePose3>("payload_pose").value();
    const auto end_effector_to_payload_transform =
        adapter_->calculateEndEffectorToPayloadTransform(end_effector_pose, payload_pose);
    setOutput("end_effector_to_payload_transform", end_effector_to_payload_transform);
    return BT::NodeStatus::SUCCESS;
  }

private:
  PickAndPlaceRobotMovementsInterface::Ptr adapter_;
};

}  // namespace tijcore
