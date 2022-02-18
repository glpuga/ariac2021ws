/* Copyright [2022] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library
#include <iostream>
#include <sstream>

// gtest
#include "gtest/gtest.h"

// project
#include <tijbt/BTExecutionResult.hpp>

namespace tijbt
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
  ASSERT_EQ(BTExecutionResult::SUCCESS, behavior_tree_result::fromString("SUCCESS"));
  ASSERT_EQ(BTExecutionResult::FAILURE, behavior_tree_result::fromString("FAILURE"));
  ASSERT_EQ(BTExecutionResult::ERROR, behavior_tree_result::fromString("ERROR"));
  ASSERT_THROW(behavior_tree_result::fromString("aasdfa"), std::invalid_argument);
}

TEST_F(AgvIdTests, TestToString)
{
  ASSERT_EQ("SUCCESS", behavior_tree_result::toString(BTExecutionResult::SUCCESS));
  ASSERT_EQ("FAILURE", behavior_tree_result::toString(BTExecutionResult::FAILURE));
  ASSERT_EQ("ERROR", behavior_tree_result::toString(BTExecutionResult::ERROR));
}

TEST_F(AgvIdTests, TestStreamOperator)
{
  std::ostringstream os;

  os << BTExecutionResult::SUCCESS << "\n" << BTExecutionResult::FAILURE << "\n" << BTExecutionResult::ERROR << "\n";

  ASSERT_EQ(
      "SUCCESS\n"
      "FAILURE\n"
      "ERROR\n",
      os.str());
}

TEST_F(AgvIdTests, TestIsValid)
{
  ASSERT_TRUE(behavior_tree_result::isValid("SUCCESS"));
  ASSERT_TRUE(behavior_tree_result::isValid("FAILURE"));
  ASSERT_TRUE(behavior_tree_result::isValid("ERROR"));

  ASSERT_FALSE(behavior_tree_result::isValid(""));
  ASSERT_FALSE(behavior_tree_result::isValid("agv0"));
  ASSERT_FALSE(behavior_tree_result::isValid("agv5"));
  ASSERT_FALSE(behavior_tree_result::isValid("foo"));
}

}  // namespace

}  // namespace test

}  // namespace tijbt
