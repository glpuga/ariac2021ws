/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <iostream>
#include <sstream>

// gtest
#include "gtest/gtest.h"

// tijcore
#include <tijcore/datatypes/PartId.hpp>

namespace tijcore
{
namespace test
{
namespace
{
using ::testing::Test;

class PartTests : public Test
{
};

TEST_F(PartTests, ConstructFromCodes)
{
  PartId uut{ PartTypeId::battery, PartColorId::red };
  ASSERT_EQ(PartTypeId::battery, uut.type());
  ASSERT_EQ(PartColorId::red, uut.color());
  ASSERT_EQ("assembly_battery_red", uut.codedString());
  ASSERT_NE(PartId::UnkownPartId, uut);
}

TEST_F(PartTests, ConstructFromString)
{
  PartId uut{ "assembly_battery_red" };
  ASSERT_EQ(PartTypeId::battery, uut.type());
  ASSERT_EQ(PartColorId::red, uut.color());
  ASSERT_EQ("assembly_battery_red", uut.codedString());
  ASSERT_NE(PartId::UnkownPartId, uut);
}

TEST_F(PartTests, InvalidConstructionStrings)
{
  // Generally bad
  EXPECT_THROW(PartId{ "" }, std::invalid_argument);
  EXPECT_THROW(PartId{ "foo" }, std::invalid_argument);
  EXPECT_THROW(PartId{ "battery_red" }, std::invalid_argument);
  EXPECT_THROW(PartId{ "assembly_red_battery" }, std::invalid_argument);
  EXPECT_THROW(PartId{ "assembly-battery-red" }, std::invalid_argument);

  // Bad prefix
  EXPECT_THROW(PartId{ "_battery_red" }, std::invalid_argument);
  EXPECT_THROW(PartId{ "foo_battery_red" }, std::invalid_argument);
  EXPECT_THROW(PartId{ "kitting_battery_red" }, std::invalid_argument);
  // Bad type
  EXPECT_THROW(PartId{ "assembly_red" }, std::invalid_argument);
  EXPECT_THROW(PartId{ "assembly__red" }, std::invalid_argument);
  EXPECT_THROW(PartId{ "assembly_gear_red" }, std::invalid_argument);
  // Bad color
  EXPECT_THROW(PartId{ "assembly_sensor_" }, std::invalid_argument);
  EXPECT_THROW(PartId{ "assembly_sensor_black" }, std::invalid_argument);
}

TEST_F(PartTests, EqualityTests)
{
  EXPECT_TRUE(PartId{ "assembly_sensor_red" } == PartId{ "assembly_sensor_red" });
  EXPECT_FALSE(PartId{ "assembly_sensor_red" } == PartId{ "assembly_sensor_blue" });
  EXPECT_FALSE(PartId{ "assembly_pump_red" } == PartId{ "assembly_sensor_red" });
  EXPECT_FALSE(PartId{ "assembly_pump_red" } == PartId{ "assembly_sensor_blue" });
}

TEST_F(PartTests, InequalityTests)
{
  EXPECT_FALSE(PartId{ "assembly_sensor_red" } != PartId{ "assembly_sensor_red" });
  EXPECT_TRUE(PartId{ "assembly_sensor_red" } != PartId{ "assembly_sensor_blue" });
  EXPECT_TRUE(PartId{ "assembly_pump_red" } != PartId{ "assembly_sensor_red" });
  EXPECT_TRUE(PartId{ "assembly_pump_red" } != PartId{ "assembly_sensor_blue" });
}

TEST_F(PartTests, UnknownPartEqualityTests)
{
  ASSERT_TRUE(PartId::UnkownPartId == PartId::UnkownPartId);
  ASSERT_TRUE(PartId::UnkownPartId != PartId{ "assembly_sensor_red" });
  ASSERT_TRUE(PartId{ "assembly_sensor_red" } != PartId::UnkownPartId);
}

}  // namespace

}  // namespace test

}  // namespace tijcore
