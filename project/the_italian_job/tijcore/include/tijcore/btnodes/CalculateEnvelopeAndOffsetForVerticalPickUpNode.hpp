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
#include <tijcore/utils/PayloadEnvelope.hpp>

namespace tijcore
{
class CalculateEnvelopeAndOffsetForVerticalPickUpNode : public BT::SyncActionNode
{
public:
  CalculateEnvelopeAndOffsetForVerticalPickUpNode(const std::string& name,
                                                  const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<BTTaskParameters::SharedPtr>("task_parameters"),
      BT::OutputPort<PayloadEnvelope>("payload_envelope"),
      BT::OutputPort<double>("offset_to_top"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto task_parameters = getInput<BTTaskParameters::SharedPtr>("task_parameters").value();
    const auto toolbox = task_parameters->toolbox;

    const auto frame_transformer = toolbox->getFrameTransformer();
    const auto scene_config = toolbox->getSceneConfigReader();

    const auto& source_resource = *task_parameters->src_locus.value().resource();
    const auto source_resource_pose = source_resource.pose();

    const auto source_resource_pose_in_world_frame = frame_transformer->transformPoseToFrame(
        source_resource_pose, scene_config->getWorldFrameId());

    PayloadEnvelope payload_envelope;
    double offset_to_top{ 0.0 };

    if (source_resource.isLocusWithPart())
    {
      const auto part_type = source_resource.qualifiedPartInfo().part_type;
      const auto part_type_id = part_type.type();
      offset_to_top = PayloadEnvelope::offsetToTop(part_type_id,
                                                   source_resource_pose_in_world_frame.rotation());
      payload_envelope = PayloadEnvelope::makeEnvelope(part_type_id);
    }
    else if (source_resource.isLocusWithMovableTray())
    {
      const auto movable_tray_id = source_resource.qualifiedMovableTrayInfo().tray_type;
      offset_to_top = PayloadEnvelope::offsetToTop(movable_tray_id);
      payload_envelope = PayloadEnvelope::makeEnvelope(movable_tray_id);
    }
    else
    {
      throw std::logic_error{ "Unknown resource type, cant determine envelope info" };
    }

    setOutput("payload_envelope", payload_envelope);
    setOutput("offset_to_top", offset_to_top);

    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace tijcore
