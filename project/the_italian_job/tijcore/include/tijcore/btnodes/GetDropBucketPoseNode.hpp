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
class GetDropBucketPoseNode : public BT::SyncActionNode
{
public:
  GetDropBucketPoseNode(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<BTTaskParameters::SharedPtr>("task_parameters"),
      BT::OutputPort<tijmath::RelativePose3>("drop_bucket_pose"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto task_parameters = getInput<BTTaskParameters::SharedPtr>("task_parameters").value();
    const auto toolbox = task_parameters->toolbox;
    const auto scene_reader = toolbox->getSceneConfigReader();
    const auto drop_bucket_pose = scene_reader->getDropBucketPose();
    setOutput("drop_bucket_pose", drop_bucket_pose);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace tijcore
