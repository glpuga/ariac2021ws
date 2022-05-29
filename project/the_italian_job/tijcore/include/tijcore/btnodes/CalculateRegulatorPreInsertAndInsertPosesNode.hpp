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
class CalculateRegulatorPreInsertAndInsertPosesNode : public BT::SyncActionNode
{
public:
  CalculateRegulatorPreInsertAndInsertPosesNode(const std::string& name,
                                                const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<BTTaskParameters::SharedPtr>("task_parameters"),
      BT::InputPort<tijmath::RelativePose3>("target_pose"),
      BT::OutputPort<tijmath::RelativePose3>("pre_insert_pose"),
      BT::OutputPort<tijmath::RelativePose3>("insert_pose"),
      BT::OutputPort<tijmath::RelativePose3>("gripper_exit_pose"),
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

    auto pre_insert_pose = target_in_world;
    pre_insert_pose.position().vector().z() += 0.06;
    pre_insert_pose.position().vector().x() += 0.011;  // TODO(glpuga) fine tuning
    setOutput("pre_insert_pose", pre_insert_pose);

    auto insert_pose = target_in_world;
    insert_pose.position().vector().z() += 0.04;
    insert_pose.position().vector().x() += 0.011;  // TODO(glpuga) fine tuning
    setOutput("insert_pose", insert_pose);

    auto gripper_exit_pose = target_in_world;
    gripper_exit_pose.position().vector().z() += 0.20;
    gripper_exit_pose.position().vector().x() += 0.05;
    setOutput("gripper_exit_pose", insert_pose);

    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace tijcore
