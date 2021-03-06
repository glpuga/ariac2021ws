/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// Standard library
#include <iostream>
#include <sstream>

// gtest
#include "gtest/gtest.h"

// tijcore
#include <tijcore/agents/PartTypeId.hpp>

namespace tijcore {

namespace test {

namespace {

using ::testing::Test;

class PartTypeIdTests : public Test {
public:
};

TEST_F(PartTypeIdTests, TestFromString) {
  ASSERT_EQ(PartTypeId::battery, part_type::fromString("battery"));
  ASSERT_EQ(PartTypeId::sensor, part_type::fromString("sensor"));
  ASSERT_EQ(PartTypeId::regulator, part_type::fromString("regulator"));
  ASSERT_EQ(PartTypeId::pump, part_type::fromString("pump"));
  ASSERT_THROW(part_type::fromString("bongo"), std::invalid_argument);
}

TEST_F(PartTypeIdTests, TestToString) {
  ASSERT_EQ("battery", part_type::toString(PartTypeId::battery));
  ASSERT_EQ("sensor", part_type::toString(PartTypeId::sensor));
  ASSERT_EQ("regulator", part_type::toString(PartTypeId::regulator));
  ASSERT_EQ("pump", part_type::toString(PartTypeId::pump));
}

TEST_F(PartTypeIdTests, TestStreamOperator) {
  std::ostringstream os;

  os << PartTypeId::battery << "\n"
     << PartTypeId::sensor << "\n"
     << PartTypeId::regulator << "\n"
     << PartTypeId::pump << "\n";

  ASSERT_EQ("battery\n"
            "sensor\n"
            "regulator\n"
            "pump\n",
            os.str());
}

TEST_F(PartTypeIdTests, TestIsValid) {
  ASSERT_TRUE(part_type::isValid("battery"));
  ASSERT_TRUE(part_type::isValid("sensor"));
  ASSERT_TRUE(part_type::isValid("regulator"));
  ASSERT_TRUE(part_type::isValid("pump"));

  ASSERT_FALSE(part_type::isValid(""));
  ASSERT_FALSE(part_type::isValid("gear"));
  ASSERT_FALSE(part_type::isValid("assembly"));
  ASSERT_FALSE(part_type::isValid("foo"));
}

} // namespace

} // namespace test

} // namespace tijcore
