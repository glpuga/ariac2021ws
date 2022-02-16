/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// gtest
#include "gtest/gtest.h"

// tijcore
#include <tijcore/datatypes/AssemblyShipment.hpp>
#include <tijcore/datatypes/KittingShipment.hpp>
#include <tijcore/datatypes/ObservedModel.hpp>
#include <tijcore/datatypes/Order.hpp>
#include <tijcore/datatypes/ProductRequest.hpp>
#include <tijcore/datatypes/ShipmentType.hpp>

namespace tijcore
{
namespace test
{
namespace
{
using ::testing::Test;

class PlainOldDataTests : public Test
{
};

TEST_F(PlainOldDataTests, ShipmentType)
{
  ShipmentType uut;
}

TEST_F(PlainOldDataTests, ProductRequest)
{
  ProductRequest uut{ PartId{ "assembly_pump_red" }, {} };
}

TEST_F(PlainOldDataTests, AssemblyShipment)
{
  AssemblyShipment uut;
}

TEST_F(PlainOldDataTests, KittingShipment)
{
  KittingShipment uut;
}

TEST_F(PlainOldDataTests, Order)
{
  Order uut{ OrderId{ "order_0_update" }, {}, {} };
}

TEST_F(PlainOldDataTests, ObservedModel)
{
  ObservedModel uut{ PartId::UnkownPartId, tijmath::RelativePose3::Origin };
}

}  // namespace

}  // namespace test

}  // namespace tijcore
