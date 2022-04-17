/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <set>
#include <string>
#include <utility>
#include <vector>

// tijcore
#include <tijcore/abstractions/OrderProcessingStrategyInterface.hpp>
#include <tijcore/abstractions/ResourceManagerInterface.hpp>
#include <tijcore/abstractions/RobotTaskFactoryInterface.hpp>
#include <tijcore/coremodels/Toolbox.hpp>

namespace tijcore
{
class OrderProcessingStrategy : public OrderProcessingStrategyInterface
{
public:
  OrderProcessingStrategy(const ResourceManagerInterface::SharedPtr& resource_manager,
                          const RobotTaskFactoryInterface::SharedPtr& robot_task_factory,
                          const Toolbox::SharedPtr& toolbox);

  void getAgvsAndStationsInUse(Order& order, std::set<AgvId>& agvs_in_use,
                               std::set<StationId>& stations_in_use) const override;

  void disambiguateOrderEndpoints(Order& order, std::set<AgvId>& agvs_in_use,
                                  std::set<StationId>& stations_in_use) const override;

  std::vector<RobotTaskInterface::Ptr>
  processOrder(Order& order, const std::set<AgvId>& agvs_in_use,
               const std::set<StationId>& stations_in_use) const override;

private:
  enum class ShipmentClass
  {
    Assembly,
    Kitting
  };

  struct InitialWorldState
  {
    std::vector<ResourceManagerInterface::ManagedLocusHandle> parts_in_place;
    std::vector<std::pair<ResourceManagerInterface::ManagedLocusHandle, PartId>> missing_parts;

    std::vector<ResourceManagerInterface::ManagedLocusHandle> broken_parts;
    std::vector<ResourceManagerInterface::ManagedLocusHandle> unwanted_parts;
  };

  ResourceManagerInterface::SharedPtr resource_manager_;
  RobotTaskFactoryInterface::SharedPtr robot_task_factory_;
  Toolbox::SharedPtr toolbox_;

  FrameTransformerInterface::SharedPtr frame_transformer_;

  std::pair<std::vector<RobotTaskInterface::Ptr>, bool> processKittingShipment(
      const OrderId& order, const KittingShipment& shipment, const std::set<AgvId>& agvs_in_use,
      const std::set<StationId>& stations_in_use) const;

  std::pair<std::vector<RobotTaskInterface::Ptr>, bool> processAssemblyShipment(
      const OrderId& order, const AssemblyShipment& shipment, const std::set<AgvId>& agvs_in_use,
      const std::set<StationId>& stations_in_use) const;

  InitialWorldState getInitialWorldState(        //
      const std::string& target_container_name,  //
      const std::vector<ProductRequest>& products) const;

  void removeLociInTargetSurfaces(                                                      //
      const std::set<AgvId>& agvs_in_use,                                               //
      const std::set<StationId>& assemblies_in_use,                                     //
      std::vector<ResourceManagerInterface::ManagedLocusHandle>& target_vector) const;  //

  std::vector<RobotTaskInterface::Ptr> stageRemoveBrokenParts(  //
      InitialWorldState& world_state) const;                    //

  std::vector<RobotTaskInterface::Ptr> stageRemoveUnwantedParts(  //
      const std::set<AgvId>& agvs_in_use,                         //
      const std::set<StationId>& assemblies_in_use,               //
      InitialWorldState& world_state) const;                      //

  std::vector<RobotTaskInterface::Ptr> stagePlaceMissingParts(  //
      const std::set<AgvId>& agvs_in_use,                       //
      const std::set<StationId>& assemblies_in_use,             //
      InitialWorldState& world_state,                           //
      int32_t& unavailable_part_count) const;                   //

  std::vector<RobotTaskInterface::Ptr> stageSubmitShipping(  //
      const std::string& target_container_name,              //
      const ShipmentClass shipment_class,                    //
      const ShipmentType& shipment_type,                     //
      const StationId& station_id,                           //
      InitialWorldState& world_state,                        //
      bool& shipping_done) const;                            //

  std::vector<RobotTaskInterface::Ptr> stageBringAMovableTray(  //
      MovableTrayId movable_tray_id,                            //
      tijmath::RelativePose3 movable_tray_pose,                 //
      const std::set<AgvId>& agvs_in_use,                       //
      const std::set<StationId>& assemblies_in_use,             //
      const std::string& target_container_name) const;

  void stableSortByDistanceToReferencePose(                                                  //
      const tijmath::RelativePose3& reference_pose,                                          //
      std::vector<ResourceManagerInterface::ManagedLocusHandle>& target_loci_vector) const;  //

  bool movableTrayIsInPlace(MovableTrayId movable_tray_id,
                            tijmath::RelativePose3 movable_tray_pose) const;
};

}  // namespace tijcore
