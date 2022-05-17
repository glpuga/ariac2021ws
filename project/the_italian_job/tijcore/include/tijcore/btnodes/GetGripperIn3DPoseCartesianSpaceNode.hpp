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
class GetGripperIn3DPoseCartesianSpaceNode : public BT::AsyncActionNode
{
public:
  GetGripperIn3DPoseCartesianSpaceNode(const std::string& name, const BT::NodeConfiguration& config,
                                       PickAndPlaceRobotMovementsInterface::Ptr adapter)
    : AsyncActionNode(name, config), adapter_{ std::move(adapter) }
  {
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<tijmath::RelativePose3>("target_pose"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto target_pose = getInput<tijmath::RelativePose3>("target_pose").value();
    const auto retval = adapter_->getGripperIn3DPoseCartesianSpace(target_pose);
    return retval ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  void halt() override
  {
    adapter_->abortCurrentAction();
  }

private:
  PickAndPlaceRobotMovementsInterface::Ptr adapter_;
};

}  // namespace tijcore
