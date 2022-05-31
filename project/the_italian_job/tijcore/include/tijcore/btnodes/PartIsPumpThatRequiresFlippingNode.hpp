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
class PartIsPumpThatRequiresFlippingNode : public BT::SyncActionNode
{
public:
  PartIsPumpThatRequiresFlippingNode(const std::string& name, const BT::NodeConfiguration& config)
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

    auto has_qualified_part_info = task_parameters->src_locus->resource()->isLocusWithPart();

    if (!has_qualified_part_info)
    {
      return BT::NodeStatus::FAILURE;
    }

    const auto src_part_type =
        task_parameters->src_locus->resource()->qualifiedPartInfo().part_type.type();

    const auto part_is_a_pump = (src_part_type == PartTypeId::pump);

    const auto src_rot_matrix =
        task_parameters->src_locus->resource()->pose().rotation().rotationMatrix();
    const auto dst_rot_matrix =
        task_parameters->dst_locus->resource()->pose().rotation().rotationMatrix();

    const auto src_z_axis = src_rot_matrix.col(2);
    const auto dst_z_axis = dst_rot_matrix.col(2);

    const auto pump_requires_flipping = (src_z_axis.dot(dst_z_axis) < 0);

    return (part_is_a_pump && pump_requires_flipping) ? BT::NodeStatus::SUCCESS :
                                                        BT::NodeStatus::FAILURE;
  }
};

}  // namespace tijcore
