/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <iostream>
#include <sstream>

// gtest
#include "gtest/gtest.h"

// tijcore
#include <tijcore/datatypes/StationId.hpp>

namespace tijcore
{
namespace test
{
namespace
{
using ::testing::Test;

class StationIdTests : public Test
{
public:
};

TEST_F(StationIdTests, TestFromString)
{
  ASSERT_EQ(StationId::as1, station_id::fromString("as1"));
  ASSERT_EQ(StationId::as2, station_id::fromString("as2"));
  ASSERT_EQ(StationId::as3, station_id::fromString("as3"));
  ASSERT_EQ(StationId::as4, station_id::fromString("as4"));
  ASSERT_EQ(StationId::ks1, station_id::fromString("ks1"));
  ASSERT_EQ(StationId::ks2, station_id::fromString("ks2"));
  ASSERT_EQ(StationId::ks3, station_id::fromString("ks3"));
  ASSERT_EQ(StationId::ks4, station_id::fromString("ks4"));
  ASSERT_EQ(StationId::any, station_id::fromString("any"));
  ASSERT_THROW(station_id::fromString("bongo"), std::invalid_argument);
}

TEST_F(StationIdTests, TestToString)
{
  ASSERT_EQ("as1", station_id::toString(StationId::as1));
  ASSERT_EQ("as2", station_id::toString(StationId::as2));
  ASSERT_EQ("as3", station_id::toString(StationId::as3));
  ASSERT_EQ("as4", station_id::toString(StationId::as4));
  ASSERT_EQ("ks1", station_id::toString(StationId::ks1));
  ASSERT_EQ("ks2", station_id::toString(StationId::ks2));
  ASSERT_EQ("ks3", station_id::toString(StationId::ks3));
  ASSERT_EQ("ks4", station_id::toString(StationId::ks4));
  ASSERT_EQ("any", station_id::toString(StationId::any));
}

TEST_F(StationIdTests, TestStreamOperator)
{
  std::ostringstream os;

  os << StationId::as1 << "\n"
     << StationId::as2 << "\n"
     << StationId::as3 << "\n"
     << StationId::as4 << "\n"
     << StationId::ks1 << "\n"
     << StationId::ks2 << "\n"
     << StationId::ks3 << "\n"
     << StationId::ks4 << "\n"
     << StationId::any << "\n";

  ASSERT_EQ(
      "as1\n"
      "as2\n"
      "as3\n"
      "as4\n"
      "ks1\n"
      "ks2\n"
      "ks3\n"
      "ks4\n"
      "any\n",
      os.str());
}

TEST_F(StationIdTests, TestIsAssembly)
{
  ASSERT_TRUE(station_id::isAssemblyStation(StationId::as1));
  ASSERT_TRUE(station_id::isAssemblyStation(StationId::as2));
  ASSERT_TRUE(station_id::isAssemblyStation(StationId::as3));
  ASSERT_TRUE(station_id::isAssemblyStation(StationId::as4));
  ASSERT_FALSE(station_id::isAssemblyStation(StationId::ks1));
  ASSERT_FALSE(station_id::isAssemblyStation(StationId::ks2));
  ASSERT_FALSE(station_id::isAssemblyStation(StationId::ks3));
  ASSERT_FALSE(station_id::isAssemblyStation(StationId::ks4));
  ASSERT_FALSE(station_id::isAssemblyStation(StationId::any));
}

TEST_F(StationIdTests, TestIsKitting)
{
  ASSERT_FALSE(station_id::isKittingStation(StationId::as1));
  ASSERT_FALSE(station_id::isKittingStation(StationId::as2));
  ASSERT_FALSE(station_id::isKittingStation(StationId::as3));
  ASSERT_FALSE(station_id::isKittingStation(StationId::as4));
  ASSERT_TRUE(station_id::isKittingStation(StationId::ks1));
  ASSERT_TRUE(station_id::isKittingStation(StationId::ks2));
  ASSERT_TRUE(station_id::isKittingStation(StationId::ks3));
  ASSERT_TRUE(station_id::isKittingStation(StationId::ks4));
  ASSERT_FALSE(station_id::isAssemblyStation(StationId::any));
}

TEST_F(StationIdTests, TestIsAny)
{
  ASSERT_FALSE(station_id::isAny(StationId::as1));
  ASSERT_FALSE(station_id::isAny(StationId::as2));
  ASSERT_FALSE(station_id::isAny(StationId::as3));
  ASSERT_FALSE(station_id::isAny(StationId::as4));
  ASSERT_FALSE(station_id::isAny(StationId::ks1));
  ASSERT_FALSE(station_id::isAny(StationId::ks2));
  ASSERT_FALSE(station_id::isAny(StationId::ks3));
  ASSERT_FALSE(station_id::isAny(StationId::ks4));
  ASSERT_TRUE(station_id::isAny(StationId::any));
}

TEST_F(StationIdTests, TestIsValid)
{
  ASSERT_TRUE(station_id::isValid("as1"));
  ASSERT_TRUE(station_id::isValid("as2"));
  ASSERT_TRUE(station_id::isValid("as3"));
  ASSERT_TRUE(station_id::isValid("as4"));
  ASSERT_TRUE(station_id::isValid("ks1"));
  ASSERT_TRUE(station_id::isValid("ks2"));
  ASSERT_TRUE(station_id::isValid("ks3"));
  ASSERT_TRUE(station_id::isValid("ks4"));

  ASSERT_FALSE(station_id::isValid(""));
  ASSERT_FALSE(station_id::isValid("as0"));
  ASSERT_FALSE(station_id::isValid("as5"));
  ASSERT_FALSE(station_id::isValid("ks0"));
  ASSERT_FALSE(station_id::isValid("ks5"));
  ASSERT_FALSE(station_id::isValid("foo"));
}

}  // namespace

}  // namespace test

}  // namespace tijcore
