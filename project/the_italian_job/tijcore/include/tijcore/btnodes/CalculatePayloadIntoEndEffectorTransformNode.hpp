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
class CalculatePayloadIntoEndEffectorTransformNode : public BT::SyncActionNode
{
public:
  CalculatePayloadIntoEndEffectorTransformNode(const std::string& name,
                                               const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<BTTaskParameters::SharedPtr>("task_parameters"),
      BT::InputPort<tijmath::RelativePose3>("end_effector_pose"),
      BT::InputPort<tijmath::RelativePose3>("payload_pose"),
      BT::OutputPort<tijmath::Isometry>("payload_into_end_effector_transform"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto end_effector_pose = getInput<tijmath::RelativePose3>("end_effector_pose").value();
    auto payload_pose = getInput<tijmath::RelativePose3>("payload_pose").value();
    auto task_parameters = getInput<BTTaskParameters::SharedPtr>("task_parameters").value();

    tijmath::Isometry payload_into_end_effector_transform;

    if (task_parameters->ee_to_payload_iso)
    {
      // we have actual data obtained from cameras
      payload_into_end_effector_transform = task_parameters->ee_to_payload_iso.value();
    }
    else
    {
      // estimate from other data
      const auto adapter_ = task_parameters->primary_robot.value().resource();
      payload_into_end_effector_transform =
          adapter_->calculatePayloadIntoEndEffectorTransform(end_effector_pose, payload_pose);
    }

    setOutput("payload_into_end_effector_transform", payload_into_end_effector_transform);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace tijcore
