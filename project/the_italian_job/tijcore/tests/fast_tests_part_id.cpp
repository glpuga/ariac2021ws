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

TEST_F(PartTests, InvalidStringsCauseConstructorToThrow)
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

TEST_F(PartTests, IsValidWorks)
{
  // Generally bad
  EXPECT_FALSE(PartId::isValid(""));
  EXPECT_FALSE(PartId::isValid("foo"));
  EXPECT_FALSE(PartId::isValid("battery_red"));
  EXPECT_FALSE(PartId::isValid("assembly_red_battery"));
  EXPECT_FALSE(PartId::isValid("assembly-battery-red"));
  // Bad prefix
  EXPECT_FALSE(PartId::isValid("_battery_red"));
  EXPECT_FALSE(PartId::isValid("foo_battery_red"));
  EXPECT_FALSE(PartId::isValid("kitting_battery_red"));
  // Bad type
  EXPECT_FALSE(PartId::isValid("assembly_red"));
  EXPECT_FALSE(PartId::isValid("assembly__red"));
  EXPECT_FALSE(PartId::isValid("assembly_gear_red"));
  // Bad color
  EXPECT_FALSE(PartId::isValid("assembly_sensor_"));
  EXPECT_FALSE(PartId::isValid("assembly_sensor_black"));

  // Good ones
  EXPECT_TRUE(PartId::isValid("assembly_battery_red"));
  EXPECT_TRUE(PartId::isValid("assembly_battery_blue"));
  EXPECT_TRUE(PartId::isValid("assembly_battery_green"));
  EXPECT_TRUE(PartId::isValid("assembly_pump_red"));
  EXPECT_TRUE(PartId::isValid("assembly_pump_blue"));
  EXPECT_TRUE(PartId::isValid("assembly_pump_green"));
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
