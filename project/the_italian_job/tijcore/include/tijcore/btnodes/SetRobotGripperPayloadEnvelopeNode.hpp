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
      BT::InputPort<BTTaskData::SharedPtr>("task_parameters"),
      BT::InputPort<PayloadEnvelope>("envelop"),
      BT::InputPort<tijmath::Pose3>("relative_pose"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto envelop = getInput<PayloadEnvelope>("envelop").value();
    auto relative_pose = getInput<tijmath::Pose3>("relative_pose").value();
    auto task_parameters = getInput<BTTaskData::SharedPtr>("task_parameters").value();
    const auto adapter_ = task_parameters->primary_robot.value().resource();
    adapter_->setRobotGripperPayloadEnvelope(envelop, relative_pose);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace tijcore
