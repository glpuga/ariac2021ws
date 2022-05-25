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
class LockAccessToVolumeAtPoseNode : public BT::SyncActionNode
{
public:
  LockAccessToVolumeAtPoseNode(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<BTTaskParameters::SharedPtr>("task_parameters"),
      BT::InputPort<tijmath::RelativePose3>("target_pose"),
      BT::InputPort<double>("radius"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto task_parameters = getInput<BTTaskParameters::SharedPtr>("task_parameters").value();
    const auto toolbox = task_parameters->toolbox;
    auto spatial_exclusion_manager = toolbox->getSpatialMutualExclusionManager();

    const auto target_pose = getInput<tijmath::RelativePose3>("target_pose").value();
    const auto radius = getInput<double>("radius").value();

    // store the handle in the task parameters structure so that it persists
    task_parameters->spatial_lock_handle =
        spatial_exclusion_manager->lockSphereVolume(target_pose, radius);

    return task_parameters->spatial_lock_handle ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace tijcore
