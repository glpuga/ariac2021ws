/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library
#include <iostream>
#include <sstream>

// gtest
#include "gtest/gtest.h"

// tijcore
#include <tijcore/agents/AgvId.hpp>

namespace tijcore
{
namespace test
{
namespace
{
using ::testing::Test;

class AgvIdTests : public Test
{
public:
};

TEST_F(AgvIdTests, TestFromString)
{
  ASSERT_EQ(AgvId::agv1, agv::fromString("agv1"));
  ASSERT_EQ(AgvId::agv2, agv::fromString("agv2"));
  ASSERT_EQ(AgvId::agv3, agv::fromString("agv3"));
  ASSERT_EQ(AgvId::agv4, agv::fromString("agv4"));
  ASSERT_EQ(AgvId::any, agv::fromString("any"));
  ASSERT_THROW(agv::fromString("agv5"), std::invalid_argument);
}

TEST_F(AgvIdTests, TestToString)
{
  ASSERT_EQ("agv1", agv::toString(AgvId::agv1));
  ASSERT_EQ("agv2", agv::toString(AgvId::agv2));
  ASSERT_EQ("agv3", agv::toString(AgvId::agv3));
  ASSERT_EQ("agv4", agv::toString(AgvId::agv4));
  ASSERT_EQ("any", agv::toString(AgvId::any));
}

TEST_F(AgvIdTests, TestStreamOperator)
{
  std::ostringstream os;

  os << AgvId::agv1 << "\n" << AgvId::agv2 << "\n" << AgvId::agv3 << "\n" << AgvId::agv4 << "\n" << AgvId::any << "\n";

  ASSERT_EQ(
      "agv1\n"
      "agv2\n"
      "agv3\n"
      "agv4\n"
      "any\n",
      os.str());
}

TEST_F(AgvIdTests, TestIsAny)
{
  ASSERT_FALSE(agv::isAny(AgvId::agv1));
  ASSERT_FALSE(agv::isAny(AgvId::agv2));
  ASSERT_FALSE(agv::isAny(AgvId::agv3));
  ASSERT_FALSE(agv::isAny(AgvId::agv4));
  ASSERT_TRUE(agv::isAny(AgvId::any));
}

TEST_F(AgvIdTests, TestIsValid)
{
  ASSERT_TRUE(agv::isValid("agv1"));
  ASSERT_TRUE(agv::isValid("agv2"));
  ASSERT_TRUE(agv::isValid("agv3"));
  ASSERT_TRUE(agv::isValid("agv4"));

  ASSERT_FALSE(agv::isValid(""));
  ASSERT_FALSE(agv::isValid("agv0"));
  ASSERT_FALSE(agv::isValid("agv5"));
  ASSERT_FALSE(agv::isValid("foo"));
}

}  // namespace

}  // namespace test

}  // namespace tijcore
