/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <string>
#include <vector>

// tijcore
#include <tijcore/datatypes/AgvId.hpp>
#include <tijcore/datatypes/MovableTrayId.hpp>
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

  MovableTrayId movable_tray_id;
  std::string movable_tray_name;
  tijmath::RelativePose3 movable_tray_pose;

  std::vector<ProductRequest> products;
};

}  // namespace tijcore
