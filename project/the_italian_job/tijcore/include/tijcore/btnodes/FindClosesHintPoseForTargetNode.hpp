/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <algorithm>
#include <string>

// external
#include "behaviortree_cpp_v3/action_node.h"

// tijcore
#include <tijcore/tasking/BTTaskParameters.hpp>

namespace tijcore
{
class FindClosesHintPoseForTargetNode : public BT::SyncActionNode
{
public:
  FindClosesHintPoseForTargetNode(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<BTTaskParameters::SharedPtr>("task_parameters"),
      BT::InputPort<tijmath::RelativePose3>("target_pose"),
      BT::OutputPort<tijmath::RelativePose3>("hint_pose"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto task_parameters = getInput<BTTaskParameters::SharedPtr>("task_parameters").value();
    const auto toolbox = task_parameters->toolbox;
    const auto scene_reader = toolbox->getSceneConfigReader();
    const auto frame_transformer = toolbox->getFrameTransformer();
    const auto world_frame_name = scene_reader->getWorldFrameId();

    auto target_pose = getInput<tijmath::RelativePose3>("target_pose").value();

    const auto target_in_world =
        frame_transformer->transformPoseToFrame(target_pose, world_frame_name);

    auto shortest_distance_to_reference_sorter =
        [& reference = target_in_world, &frame_transformer,
         &world_frame_name](const SceneConfigReaderInterface::PickAndPlacePoseHintsData& lhs,
                            const SceneConfigReaderInterface::PickAndPlacePoseHintsData& rhs) {
          const auto lhs_in_world =
              frame_transformer->transformPoseToFrame(lhs.target_pose, world_frame_name);
          const auto rhs_in_world =
              frame_transformer->transformPoseToFrame(rhs.target_pose, world_frame_name);

          auto distance_vector_left =
              (reference.position().vector() - lhs_in_world.position().vector());
          auto distance_vector_right =
              (reference.position().vector() - rhs_in_world.position().vector());

          // ignore height differences
          distance_vector_left.z() = 0.0;
          distance_vector_right.z() = 0.0;

          const auto squared_distance_left = distance_vector_left.norm();
          const auto squared_distance_right = distance_vector_right.norm();

          return squared_distance_left < squared_distance_right;
        };

    const auto robot_name = task_parameters->primary_robot->resource()->getRobotName();
    const auto hint_poses_data = scene_reader->getListOfApproachHints(robot_name);

    const auto closest_hint_it = std::min_element(hint_poses_data.begin(), hint_poses_data.end(),
                                                  shortest_distance_to_reference_sorter);

    const auto hint_pose = closest_hint_it->approach_pose;
    setOutput("hint_pose", hint_pose);

    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace tijcore
