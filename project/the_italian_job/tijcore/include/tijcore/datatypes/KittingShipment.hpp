/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <vector>

// tijcore
#include <tijcore/datatypes/AgvId.hpp>
#include <tijcore/datatypes/ProductRequest.hpp>
#include <tijcore/datatypes/ShipmentType.hpp>
#include <tijcore/datatypes/StationId.hpp>

namespace tijcore
{
struct KittingShipment
{
  ShipmentType shipment_type;

  AgvId agv_id;

  StationId station_id;

  std::vector<ProductRequest> products;
};

}  // namespace tijcore
