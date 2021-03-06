/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// gtest
#include "gtest/gtest.h"

// tijcore
#include <tijcore/agents/AssemblyShipment.hpp>
#include <tijcore/agents/KittingShipment.hpp>
#include <tijcore/agents/ObservedModel.hpp>
#include <tijcore/agents/Order.hpp>
#include <tijcore/agents/ProductRequest.hpp>
#include <tijcore/agents/ShipmentType.hpp>

namespace tijcore {

namespace test {

namespace {

using ::testing::Test;

class PlainOldDataTests : public Test {};

TEST_F(PlainOldDataTests, ShipmentType) { ShipmentType uut; }

TEST_F(PlainOldDataTests, ProductRequest) {
  ProductRequest uut{PartId{"assembly_pump_red"}, {}};
}

TEST_F(PlainOldDataTests, AssemblyShipment) { AssemblyShipment uut; }

TEST_F(PlainOldDataTests, KittingShipment) { KittingShipment uut; }

TEST_F(PlainOldDataTests, Order) {
  Order uut{OrderId{"order_0_update"}, {}, {}};
}

TEST_F(PlainOldDataTests, ObservedModel) {
  ObservedModel uut{PartId::UnkownPartId, RelativePose3::Origin};
}

} // namespace

} // namespace test

} // namespace tijcore
