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
#include <tijmath/RelativePose3.hpp>

namespace tijcore
{
class PosesAreWithinRangeNode : public BT::SyncActionNode
{
public:
  PosesAreWithinRangeNode(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<tijmath::RelativePose3>("start_pose"),
      BT::InputPort<tijmath::RelativePose3>("end_pose"),
      BT::InputPort<double>("max_range"),
      BT::InputPort<BTTaskParameters::SharedPtr>("task_parameters"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto task_parameters = getInput<BTTaskParameters::SharedPtr>("task_parameters").value();
    const auto toolbox = task_parameters->toolbox;
    const auto frame_transformer = toolbox->getFrameTransformer();
    const auto scene_config = toolbox->getSceneConfigReader();

    const auto start_pose = getInput<tijmath::RelativePose3>("start_pose").value();
    const auto end_pose = getInput<tijmath::RelativePose3>("end_pose").value();

    const auto start_pose_in_world =
        frame_transformer->transformPoseToFrame(start_pose, scene_config->getWorldFrameId());
    const auto end_pose_in_world =
        frame_transformer->transformPoseToFrame(end_pose, scene_config->getWorldFrameId());

    const auto max_range = getInput<double>("max_range").value();

    const auto distance =
        (start_pose_in_world.position().vector() - end_pose_in_world.position().vector()).norm();

    return (max_range > distance) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace tijcore
