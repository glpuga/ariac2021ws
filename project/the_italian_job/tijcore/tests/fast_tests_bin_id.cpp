/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library
#include <iostream>
#include <sstream>

// gtest
#include "gtest/gtest.h"

// tijcore
#include <tijcore/agents/BinId.hpp>

namespace tijcore
{
namespace test
{
namespace
{
using ::testing::Test;

class BinIdTests : public Test
{
public:
};

TEST_F(BinIdTests, TestFromString)
{
  ASSERT_EQ(BinId::bin1, bin::fromString("bin1"));
  ASSERT_EQ(BinId::bin2, bin::fromString("bin2"));
  ASSERT_EQ(BinId::bin3, bin::fromString("bin3"));
  ASSERT_EQ(BinId::bin4, bin::fromString("bin4"));
  ASSERT_EQ(BinId::bin5, bin::fromString("bin5"));
  ASSERT_EQ(BinId::bin6, bin::fromString("bin6"));
  ASSERT_EQ(BinId::bin7, bin::fromString("bin7"));
  ASSERT_EQ(BinId::bin8, bin::fromString("bin8"));
  ASSERT_THROW(bin::fromString("bongo"), std::invalid_argument);
}

TEST_F(BinIdTests, TestToString)
{
  ASSERT_EQ("bin1", bin::toString(BinId::bin1));
  ASSERT_EQ("bin2", bin::toString(BinId::bin2));
  ASSERT_EQ("bin3", bin::toString(BinId::bin3));
  ASSERT_EQ("bin4", bin::toString(BinId::bin4));
  ASSERT_EQ("bin5", bin::toString(BinId::bin5));
  ASSERT_EQ("bin6", bin::toString(BinId::bin6));
  ASSERT_EQ("bin7", bin::toString(BinId::bin7));
  ASSERT_EQ("bin8", bin::toString(BinId::bin8));
}

TEST_F(BinIdTests, TestStreamOperator)
{
  std::ostringstream os;

  os << BinId::bin1 << "\n"
     << BinId::bin2 << "\n"
     << BinId::bin3 << "\n"
     << BinId::bin4 << "\n"
     << BinId::bin5 << "\n"
     << BinId::bin6 << "\n"
     << BinId::bin7 << "\n"
     << BinId::bin8 << "\n";

  ASSERT_EQ(
      "bin1\n"
      "bin2\n"
      "bin3\n"
      "bin4\n"
      "bin5\n"
      "bin6\n"
      "bin7\n"
      "bin8\n",
      os.str());
}

TEST_F(BinIdTests, TestIsValid)
{
  ASSERT_TRUE(bin::isValid("bin1"));
  ASSERT_TRUE(bin::isValid("bin2"));
  ASSERT_TRUE(bin::isValid("bin3"));
  ASSERT_TRUE(bin::isValid("bin4"));
  ASSERT_TRUE(bin::isValid("bin5"));
  ASSERT_TRUE(bin::isValid("bin6"));
  ASSERT_TRUE(bin::isValid("bin7"));
  ASSERT_TRUE(bin::isValid("bin8"));

  ASSERT_FALSE(bin::isValid(""));
  ASSERT_FALSE(bin::isValid("bin0"));
  ASSERT_FALSE(bin::isValid("bin9"));
  ASSERT_FALSE(bin::isValid("foo"));
}

}  // namespace

}  // namespace test

}  // namespace tijcore
