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
#include <tijcore/abstractions/ResourceManagerInterface.hpp>
#include <tijcore/tasking/BTTaskParameters.hpp>

namespace tijcore
{
class CalculatePayloadIntoEndEffectorTransformFromCamerasIfPossibleNode : public BT::SyncActionNode
{
public:
  CalculatePayloadIntoEndEffectorTransformFromCamerasIfPossibleNode(
      const std::string& name, const BT::NodeConfiguration& config)
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

    auto toolbox = task_parameters->toolbox.get();
    auto unfiltered_model_perception_chain = toolbox->getUnfilteredModelPerceptionChain();
    auto frame_transformer = toolbox->getFrameTransformer();
    auto scene_reader = toolbox->getSceneConfigReader();
    const auto world_frame = scene_reader->getWorldFrameId();
    const auto adapter_ = task_parameters->primary_robot.value().resource();

    const auto end_effector_pose = adapter_->getCurrentEndEffectorPose();

    tijmath::RelativePose3 closest_part_pose;

    // the following scope is only to find the pose of the part in view that's closest to the
    // gripper
    {
      auto shortest_distance_to_pose = [&](const ObservedItem& lhs_locus,
                                           const ObservedItem& rhs_locus) {
        // get all poses in the same frame
        auto lhs_pose =
            frame_transformer->transformPoseToFrame(lhs_locus.pose, end_effector_pose.frameId());
        auto rhs_pose =
            frame_transformer->transformPoseToFrame(rhs_locus.pose, end_effector_pose.frameId());

        const auto lhs_distance =
            (lhs_pose.position().vector() - end_effector_pose.position().vector()).norm();
        const auto rhs_distance =
            (rhs_pose.position().vector() - end_effector_pose.position().vector()).norm();

        return lhs_distance < rhs_distance;
      };

      auto visible_models = unfiltered_model_perception_chain->getObservedModels();

      auto is_not_a_part = [&](const ObservedItem& item) {
        return !item.item.is<QualifiedPartInfo>();
      };

      visible_models.erase(
          std::remove_if(visible_models.begin(), visible_models.end(), is_not_a_part),
          visible_models.end());

      if (visible_models.size() == 0)
      {
        WARNING("No parts found in the scene, so the source part cannot be calibrated.");
        return BT::NodeStatus::SUCCESS;
      }

      auto closest_part =
          std::min_element(visible_models.begin(), visible_models.end(), shortest_distance_to_pose);

      closest_part_pose =
          frame_transformer->transformPoseToFrame(closest_part->pose, end_effector_pose.frameId());

      const auto calibration_distance =
          (closest_part_pose.position().vector() - end_effector_pose.position().vector()).norm();

      if (calibration_distance > 0.2)
      {
        WARNING(
            "The closest visible part in the scene is not close enough ({} m), and will be "
            "ignored.",
            calibration_distance);
        return BT::NodeStatus::SUCCESS;
      }
    }

    // we calculate the calibrated ee_to_payload pose as view through the cameras
    task_parameters->ee_to_payload_iso =
        adapter_->calculatePayloadIntoEndEffectorTransform(end_effector_pose, closest_part_pose);

    return BT::NodeStatus::SUCCESS;
  }
};  // namespace tijcore

}  // namespace tijcore
