/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <stdexcept>
#include <string>
// external
#include "behaviortree_cpp_v3/action_node.h"

// tijcore
#include <tijcore/tasking/BTTaskParameters.hpp>

namespace tijcore
{
class ManeouverTypeIsNode : public BT::SyncActionNode
{
public:
  ManeouverTypeIsNode(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<BTTaskParameters::SharedPtr>("task_parameters"),
      BT::InputPort<std::string>("type"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto task_parameters = getInput<BTTaskParameters::SharedPtr>("task_parameters").value();
    auto type = getInput<std::string>("type").value();

    const auto dst_parent_name = task_parameters->dst_locus->resource()->parentName();
    const auto src_part_type =
        task_parameters->src_locus->resource()->qualifiedPartInfo().part_type.type();

    auto is_a_valid_assembly_station =
        station_id::isValid(dst_parent_name) &&
        station_id::isAssemblyStation(station_id::fromString(dst_parent_name));

    if (type == "SensorAssembly")
    {
      if (is_a_valid_assembly_station && (src_part_type == PartTypeId::sensor))
      {
        return BT::NodeStatus::SUCCESS;
      }
    }
    else if (type == "RegulatorAssembly")
    {
      if (is_a_valid_assembly_station && (src_part_type == PartTypeId::regulator))
      {
        return BT::NodeStatus::SUCCESS;
      }
    }
    else if (type == "PlaceFromAbove")
    {
      // anything that's not a sensor or regulator in an assembly station can be placedfrom above
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      throw std::runtime_error("Unknown placement type requested: " + type);
    }

    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace tijcore
