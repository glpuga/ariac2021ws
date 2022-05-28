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
class GetGripperIn3DPoseCartesianSpaceNode : public BT::AsyncActionNode
{
public:
  GetGripperIn3DPoseCartesianSpaceNode(const std::string& name, const BT::NodeConfiguration& config)
    : AsyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<BTTaskParameters::SharedPtr>("task_parameters"),
      BT::InputPort<tijmath::RelativePose3>("target_pose"),
      BT::InputPort<double>("dynamic_factor"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto task_parameters = getInput<BTTaskParameters::SharedPtr>("task_parameters").value();
    const auto adapter_ = task_parameters->primary_robot.value().resource();

    auto target_pose = getInput<tijmath::RelativePose3>("target_pose").value();

    auto dynamic_factor_opt = getInput<double>("dynamic_factor");
    const double dynamic_factor = dynamic_factor_opt.has_value() ? dynamic_factor_opt.value() : 1.0;

    const auto retval = adapter_->getGripperIn3DPoseCartesianSpace(target_pose, dynamic_factor);

    return retval ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  void halt() override
  {
    // auto task_parameters = getInput<BTTaskParameters::SharedPtr>("task_parameters").value();
    // const auto adapter_ = task_parameters->primary_robot.value().resource();
    // adapter_->abortCurrentAction();
  }
};

}  // namespace tijcore
