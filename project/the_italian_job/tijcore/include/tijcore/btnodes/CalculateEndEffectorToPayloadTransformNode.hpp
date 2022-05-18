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
class CalculateEndEffectorToPayloadTransformNode : public BT::SyncActionNode
{
public:
  CalculateEndEffectorToPayloadTransformNode(const std::string& name,
                                             const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<BTTaskData::SharedPtr>("task_parameters"),
      BT::InputPort<tijmath::RelativePose3>("end_effector_pose"),
      BT::InputPort<tijmath::RelativePose3>("payload_pose"),
      BT::OutputPort<tijmath::Pose3>("end_effector_to_payload_transform"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto end_effector_pose = getInput<tijmath::RelativePose3>("end_effector_pose").value();
    auto payload_pose = getInput<tijmath::RelativePose3>("payload_pose").value();
    auto task_parameters = getInput<BTTaskData::SharedPtr>("task_parameters").value();
    const auto adapter_ = task_parameters->primary_robot.value().resource();
    const auto end_effector_to_payload_transform =
        adapter_->calculateEndEffectorToPayloadTransform(end_effector_pose, payload_pose);
    setOutput("end_effector_to_payload_transform", end_effector_to_payload_transform);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace tijcore
