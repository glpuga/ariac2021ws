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
#include <tijcore/utils/PayloadEnvelope.hpp>

namespace tijcore
{
class SetRobotGripperPayloadEnvelopeNode : public BT::SyncActionNode
{
public:
  SetRobotGripperPayloadEnvelopeNode(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<BTTaskParameters::SharedPtr>("task_parameters"),
      BT::InputPort<PayloadEnvelope>("payload_envelope"),
      BT::InputPort<tijmath::Isometry>("payload_into_end_effector_transform"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto payload_envelope = getInput<PayloadEnvelope>("payload_envelope").value();
    auto payload_into_end_effector_transform =
        getInput<tijmath::Isometry>("payload_into_end_effector_transform").value();
    auto task_parameters = getInput<BTTaskParameters::SharedPtr>("task_parameters").value();
    const auto adapter_ = task_parameters->primary_robot.value().resource();
    adapter_->setRobotGripperPayloadEnvelope(payload_envelope, payload_into_end_effector_transform);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace tijcore
