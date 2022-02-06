/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <vector>

// tijcore
#include <tijcore/agents/ProductRequest.hpp>
#include <tijcore/agents/ShipmentType.hpp>
#include <tijcore/agents/StationId.hpp>

namespace tijcore
{
struct AssemblyShipment
{
  ShipmentType shipment_type;

  StationId station_id;

  std::vector<ProductRequest> products;
};

}  // namespace tijcore
