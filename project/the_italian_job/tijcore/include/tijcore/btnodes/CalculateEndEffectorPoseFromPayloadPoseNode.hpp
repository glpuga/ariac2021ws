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
#include <tijmath/Isometry.hpp>
#include <tijmath/RelativePose3.hpp>

namespace tijcore
{
class CalculateEndEffectorPoseFromPayloadPoseNode : public BT::SyncActionNode
{
public:
  CalculateEndEffectorPoseFromPayloadPoseNode(const std::string& name,
                                              const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<tijmath::RelativePose3>("payload_pose"),
      BT::InputPort<tijmath::Isometry>("payload_into_end_effector_transform"),
      BT::OutputPort<tijmath::RelativePose3>("end_effector_pose"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto payload_pose = getInput<tijmath::RelativePose3>("payload_pose").value();

    const auto payload_into_end_effector_transform =
        getInput<tijmath::Isometry>("payload_into_end_effector_transform").value();

    const auto payload_frame = payload_pose.frameId();

    const tijmath::Isometry payload_into_global_transform{ payload_pose.rotation().rotationMatrix(),
                                                           payload_pose.position().vector() };

    const auto end_effector_into_global_frame_transform =
        payload_into_global_transform * payload_into_end_effector_transform.inv();

    tijmath::RelativePose3 end_effector_in_global_frame_pose{
      payload_frame, tijmath::Position{ end_effector_into_global_frame_transform.translation() },
      tijmath::Rotation{ end_effector_into_global_frame_transform.rotation() }
    };

    setOutput("end_effector_pose", end_effector_in_global_frame_pose);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace tijcore
