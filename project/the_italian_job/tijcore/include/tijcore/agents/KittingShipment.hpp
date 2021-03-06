/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <vector>

// tijcore
#include <tijcore/agents/AgvId.hpp>
#include <tijcore/agents/ProductRequest.hpp>
#include <tijcore/agents/ShipmentType.hpp>
#include <tijcore/agents/StationId.hpp>

namespace tijcore {

struct KittingShipment {
  ShipmentType shipment_type;

  AgvId agv_id;

  StationId station_id;

  std::vector<ProductRequest> products;
};

} // namespace tijcore
