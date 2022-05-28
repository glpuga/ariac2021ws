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
class DestinationSurfaceIsAnAssemblyStationNode : public BT::SyncActionNode
{
public:
  DestinationSurfaceIsAnAssemblyStationNode(const std::string& name,
                                            const BT::NodeConfiguration& config)
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

    const auto dst_parent_name = task_parameters->dst_locus->resource()->parentName();

    auto is_a_valid_assembly_station =
        station_id::isValid(dst_parent_name) &&
        station_id::isAssemblyStation(station_id::fromString(dst_parent_name));

    return is_a_valid_assembly_station ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace tijcore
