/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <set>
#include <utility>
#include <vector>

// tijcore
#include <tijcore/perception/OrderProcessingStrategyInterface.hpp>
#include <tijcore/perception/ResourceManagerInterface.hpp>
#include <tijcore/perception/RobotTaskFactoryInterface.hpp>
#include <tijcore/perception/Toolbox.hpp>

namespace tijcore {

class OrderProcessingStrategy : public OrderProcessingStrategyInterface {
public:
  OrderProcessingStrategy(
      const ResourceManagerInterface::SharedPtr &resource_manager,
      const RobotTaskFactoryInterface::SharedPtr &robot_task_factory,
      const Toolbox::SharedPtr &toolbox);

  void
  getAgvsAndStationsInUse(Order &order, std::set<AgvId> &agvs_in_use,
                          std::set<StationId> &stations_in_use) const override;

  void disambiguateOrderEndpoints(
      Order &order, std::set<AgvId> &agvs_in_use,
      std::set<StationId> &stations_in_use) const override;

  std::vector<RobotTaskInterface::Ptr>
  processOrder(Order &order, const std::set<AgvId> &agvs_in_use,
               const std::set<StationId> &stations_in_use) const override;

private:
  enum class ShipmentClass { Assembly, Kitting };

  ResourceManagerInterface::SharedPtr resource_manager_;
  RobotTaskFactoryInterface::SharedPtr robot_task_factory_;
  Toolbox::SharedPtr toolbox_;

  FrameTransformerInterface::SharedPtr frame_transformer_;

  std::pair<std::vector<RobotTaskInterface::Ptr>, bool>
  processUniversalShipment(const ShipmentClass shipment_class,
                           const OrderId &order,
                           const ShipmentType &shipment_type,
                           const std::optional<AgvId> &agv_id,
                           const StationId &station_id,
                           const std::vector<ProductRequest> &products,
                           const std::set<AgvId> &agvs_in_use,
                           const std::set<StationId> &stations_in_use) const;

  std::pair<std::vector<RobotTaskInterface::Ptr>, bool>
  processKittingShipment(const OrderId &order, const KittingShipment &shipment,
                         const std::set<AgvId> &agvs_in_use,
                         const std::set<StationId> &stations_in_use) const;

  std::pair<std::vector<RobotTaskInterface::Ptr>, bool>
  processAssemblyShipment(const OrderId &order,
                          const AssemblyShipment &shipment,
                          const std::set<AgvId> &agvs_in_use,
                          const std::set<StationId> &stations_in_use) const;
};

} // namespace tijcore
