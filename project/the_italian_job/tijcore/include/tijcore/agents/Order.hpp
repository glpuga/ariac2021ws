/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <vector>

// tijcore
#include <tijcore/agents/AssemblyShipment.hpp>
#include <tijcore/agents/KittingShipment.hpp>
#include <tijcore/agents/OrderId.hpp>

namespace tijcore {

struct Order {
  OrderId order_id;

  std::vector<KittingShipment> kitting_shipments;
  std::vector<AssemblyShipment> assembly_shipments;
};

} // namespace tijcore
