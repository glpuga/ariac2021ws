/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <string>

// external
#include <behavior_tree_extras/BehaviorTreeBuilderInterface.hpp>

// tijcore
#include <tijcore/abstractions/RobotTaskFactoryInterface.hpp>
#include <tijcore/coremodels/Toolbox.hpp>
#include <tijcore/resources/ResourceManager.hpp>
namespace tijcore
{
class BTRobotTaskFactory : public RobotTaskFactoryInterface
{
public:
  BTRobotTaskFactory(const std::string& behavior_file_path,
                     const ResourceManagerInterface::SharedPtr& resource_manager,
                     const Toolbox::SharedPtr& toolbox);

  RobotTaskInterface::Ptr
  getRemoveBrokenPartTask(ResourceManagerInterface::ManagedLocusHandle&& src_locus,
                          ResourceManagerInterface::PickAndPlaceRobotHandle&& robot) const override;

  RobotTaskInterface::Ptr
  getPickAndPlacePartTask(ResourceManagerInterface::ManagedLocusHandle&& src_locus,
                          ResourceManagerInterface::ManagedLocusHandle&& dst_locus,
                          ResourceManagerInterface::PickAndPlaceRobotHandle&& robot) const override;

  RobotTaskInterface::Ptr getPickAndPlaceMovableTrayTask(
      ResourceManagerInterface::ManagedLocusHandle&& src_locus,
      ResourceManagerInterface::ManagedLocusHandle&& dst_locus,
      ResourceManagerInterface::PickAndPlaceRobotHandle&& robot) const override;

  RobotTaskInterface::Ptr getSubmitKittingShipmentTask(
      const std::string& kitting_tray_name, const StationId& destination_station,
      const ShipmentType& shipment_type) const override;

  RobotTaskInterface::Ptr getSubmitAssemblyShipmentTask(
      const std::string& assembly_tray_name, const ShipmentType& shipment_type) const override;

private:
  std::string behavior_file_path_;
  ResourceManagerInterface::SharedPtr resource_manager_;
  Toolbox::SharedPtr toolbox_;

  behavior_tree_extras::BehaviorTreeBuilderInterface::Ptr behavior_tree_base_builder_;
};

}  // namespace tijcore
