/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <stdexcept>
#include <string>

// external
#include "behaviortree_cpp_v3/action_node.h"

// tijcore
#include <tijcore/datatypes/BinId.hpp>
#include <tijcore/datatypes/StationId.hpp>
#include <tijcore/tasking/BTTaskParameters.hpp>

namespace tijcore
{
class WeAreInASharedWorkspaceNode : public BT::SyncActionNode
{
public:
  WeAreInASharedWorkspaceNode(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<BTTaskParameters::SharedPtr>("task_parameters"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto task_parameters = getInput<BTTaskParameters::SharedPtr>("task_parameters").value();
    const auto adapter_ = task_parameters->primary_robot.value().resource();

    const auto toolbox = task_parameters->toolbox;
    const auto frame_transformer = toolbox->getFrameTransformer();
    const auto scene_config = toolbox->getSceneConfigReader();

    const auto current_pose = adapter_->getCurrentRobotPose();

    const auto current_pose_in_world =
        frame_transformer->transformPoseToFrame(current_pose, scene_config->getWorldFrameId());

    const auto current_pose_in_world_x = current_pose_in_world.position().vector().x();

    const auto we_are_within_a_shared_workspace =
        (-3.0 < current_pose_in_world_x) && (current_pose_in_world_x < 1.0);

    return we_are_within_a_shared_workspace ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace tijcore
