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
class SetRobotGripperPayloadEnvelopeNode : public BT::SyncActionNode
{
public:
  SetRobotGripperPayloadEnvelopeNode(const std::string& name, const BT::NodeConfiguration& config,
                                     PickAndPlaceRobotMovementsInterface::Ptr adapter)
    : SyncActionNode(name, config), adapter_{ std::move(adapter) }
  {
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<tijcore::PayloadEnvelope>("envelop"),
      BT::InputPort<tijmath::Pose3>("relative_pose"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto envelop = getInput<tijcore::PayloadEnvelope>("envelop").value();
    auto relative_pose = getInput<tijmath::Pose3>("relative_pose").value();
    adapter_->setRobotGripperPayloadEnvelope(envelop, relative_pose);
    return BT::NodeStatus::SUCCESS;
  }

private:
  PickAndPlaceRobotMovementsInterface::Ptr adapter_;
};

}  // namespace tijcore
