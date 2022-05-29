/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <cmath>
#include <random>
#include <string>

// external
#include "behaviortree_cpp_v3/action_node.h"

// tijcore
#include <tijcore/tasking/BTTaskParameters.hpp>
#include <tijmath/RelativePose3.hpp>

namespace tijcore
{
class RandomizeTargetPoseNode : public BT::SyncActionNode
{
public:
  RandomizeTargetPoseNode(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<BTTaskParameters::SharedPtr>("task_parameters"),
      BT::InputPort<double>("max_radius"),
      BT::InputPort<tijmath::RelativePose3>("target_pose"),
      BT::OutputPort<tijmath::RelativePose3>("randomized_target_pose"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto task_parameters = getInput<BTTaskParameters::SharedPtr>("task_parameters").value();
    const auto toolbox = task_parameters->toolbox;
    const auto frame_transformer = toolbox->getFrameTransformer();
    const auto scene_config = toolbox->getSceneConfigReader();

    const auto target_pose = getInput<tijmath::RelativePose3>("target_pose").value();
    const auto max_radius = getInput<double>("max_radius").value();

    auto randomize_pose_in_world =
        frame_transformer->transformPoseToFrame(target_pose, scene_config->getWorldFrameId());

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> unif_dist(-4, 3);

    const auto distance = max_radius;
    const auto angle = static_cast<double>(unif_dist(gen)) * 2 * M_PI / 8.0;

    const auto x_diff = distance * std::cos(angle);
    const auto y_diff = distance * std::sin(angle);

    randomize_pose_in_world.position().vector().x() += x_diff;
    randomize_pose_in_world.position().vector().y() += y_diff;

    setOutput("randomized_target_pose", randomize_pose_in_world);

    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace tijcore
