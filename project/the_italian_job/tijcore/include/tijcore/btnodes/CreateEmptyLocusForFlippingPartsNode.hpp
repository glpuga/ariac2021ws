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
class CreateEmptyLocusForFlippingPartsNode : public BT::SyncActionNode
{
public:
  CreateEmptyLocusForFlippingPartsNode(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<BTTaskParameters::SharedPtr>("task_parameters"),
      BT::InputPort<tijmath::RelativePose3>("reference_pose"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto task_parameters = getInput<BTTaskParameters::SharedPtr>("task_parameters").value();
    auto reference_pose = getInput<tijmath::RelativePose3>("reference_pose").value();

    const auto robot = task_parameters->primary_robot.value().resource();

    auto resource_manager = task_parameters->resource_manager.get();
    auto toolbox = task_parameters->toolbox.get();
    auto frame_transformer = toolbox->getFrameTransformer();
    auto scene_reader = toolbox->getSceneConfigReader();
    const auto world_frame = scene_reader->getWorldFrameId();

    auto candidates = resource_manager->findVacantLociCandidates(flipping_radius_);

    auto is_reachable = [robot](const ResourceManagerInterface::ManagedLocusHandle& locus) {
      return !robot->testIfRobotReachesPose(locus.resource()->pose());
    };

    auto shortest_distance_to_pose =
        [&](const ResourceManagerInterface::ManagedLocusHandle& lhs_locus_handle,
            const ResourceManagerInterface::ManagedLocusHandle& rhs_locus_handle) {
          auto& lhs_locus = *lhs_locus_handle.resource();
          auto& rhs_locus = *rhs_locus_handle.resource();

          // get all poses in the same frame
          auto lhs_pose =
              frame_transformer->transformPoseToFrame(lhs_locus.pose(), reference_pose.frameId());
          auto rhs_pose =
              frame_transformer->transformPoseToFrame(rhs_locus.pose(), reference_pose.frameId());

          // ignore differences in height
          lhs_pose.position().vector().z() = reference_pose.position().vector().z();
          rhs_pose.position().vector().z() = reference_pose.position().vector().z();

          const auto lhs_distance =
              (lhs_pose.position().vector() - reference_pose.position().vector()).norm();
          const auto rhs_distance =
              (rhs_pose.position().vector() - reference_pose.position().vector()).norm();

          return lhs_distance < rhs_distance;
        };

    candidates.erase(std::remove_if(candidates.begin(), candidates.end(), is_reachable),
                     candidates.end());

    auto closest_part =
        std::min_element(candidates.begin(), candidates.end(), shortest_distance_to_pose);

    task_parameters->aux_locus = *closest_part;

    return BT::NodeStatus::SUCCESS;
  }

private:
  const double flipping_radius_ = 0.23;
};

}  // namespace tijcore
