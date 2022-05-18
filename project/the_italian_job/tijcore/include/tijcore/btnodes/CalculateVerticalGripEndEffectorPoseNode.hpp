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
class CalculateVerticalGripEndEffectorPoseNode : public BT::SyncActionNode
{
public:
  CalculateVerticalGripEndEffectorPoseNode(const std::string& name,
                                           const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<BTTaskData::SharedPtr>("task_parameters"),
      BT::InputPort<tijmath::RelativePose3>("target_pose"),
      BT::InputPort<tijmath::RelativePose3>("offset_to_top"),
      BT::OutputPort<tijmath::RelativePose3>("end_effector_pose"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto target_pose = getInput<tijmath::RelativePose3>("target_pose").value();
    auto offset_to_top = getInput<double>("offset_to_top").value();
    auto task_parameters = getInput<BTTaskData::SharedPtr>("task_parameters").value();
    const auto adapter_ = task_parameters->primary_robot.value().resource();
    const auto end_effector_pose =
        adapter_->calculateVerticalGripEndEffectorPose(target_pose, offset_to_top);
    setOutput("end_effector_pose", end_effector_pose);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace tijcore
