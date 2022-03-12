/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <vector>

// tijcore
#include <tijcore/datatypes/AssemblyShipment.hpp>
#include <tijcore/datatypes/KittingShipment.hpp>
#include <tijcore/datatypes/OrderId.hpp>

namespace tijcore
{
struct Order
{
  OrderId order_id;

  std::vector<KittingShipment> kitting_shipments;
  std::vector<AssemblyShipment> assembly_shipments;
};

}  // namespace tijcore
