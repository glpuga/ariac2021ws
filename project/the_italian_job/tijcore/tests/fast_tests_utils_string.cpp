/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// gtest
#include "gtest/gtest.h"

// tijcore
#include <tijcore/utils/string.hpp>

namespace tijcore
{
namespace utils
{
namespace test
{
namespace
{
using ::testing::Test;

class UtilsTests : public Test
{
};

class SplitStringTests : public UtilsTests
{
};

TEST_F(SplitStringTests, EmptyStringCase)
{
  const std::string input{ "" };
  const std::vector<std::string> expected{};
  ASSERT_EQ(expected, string::splitStringByChar(input, '-'));
}

TEST_F(SplitStringTests, SingleItem)
{
  const std::string input{ "foo" };
  const std::vector<std::string> expected{
    "foo",
  };
  ASSERT_EQ(expected, string::splitStringByChar(input, '-'));
}

TEST_F(SplitStringTests, MultipleItems)
{
  const std::string input{ "1-2-3" };
  const std::vector<std::string> expected{
    "1",
    "2",
    "3",
  };
  ASSERT_EQ(expected, string::splitStringByChar(input, '-'));
}

TEST_F(SplitStringTests, EmptyItemsStart)
{
  const std::string input{ "-1-2-3" };
  const std::vector<std::string> expected{
    "",
    "1",
    "2",
    "3",
  };
  ASSERT_EQ(expected, string::splitStringByChar(input, '-'));
}

TEST_F(SplitStringTests, EmptyItemsMiddle)
{
  const std::string input{ "1-2---3" };
  const std::vector<std::string> expected{
    "1", "2", "", "", "3",
  };
  ASSERT_EQ(expected, string::splitStringByChar(input, '-'));
}

TEST_F(SplitStringTests, EmptyItemsEnd)
{
  const std::string input{ "1-2-3--" };
  const std::vector<std::string> expected{ "1", "2", "3", "", "" };
  ASSERT_EQ(expected, string::splitStringByChar(input, '-'));
}

class JoinStringsTests : public UtilsTests
{
};

TEST_F(JoinStringsTests, EmptyVectorCase)
{
  const std::vector<std::string> input{};
  const std::string expected{ "" };
  ASSERT_EQ(expected, string::joinStringsWithSeparator(input, '-'));
  ASSERT_EQ(expected, string::joinStringsWithSeparator(input, "-"));
}

TEST_F(JoinStringsTests, SingleStringCase)
{
  const std::vector<std::string> input{ "foo" };
  const std::string expected{ "foo" };
  ASSERT_EQ(expected, string::joinStringsWithSeparator(input, '-'));
  ASSERT_EQ(expected, string::joinStringsWithSeparator(input, "-"));
}

TEST_F(JoinStringsTests, MultipleStringsCase)
{
  const std::vector<std::string> input{ "1", "2", "3" };
  const std::string expected{ "1-2-3" };
  ASSERT_EQ(expected, string::joinStringsWithSeparator(input, '-'));
  ASSERT_EQ(expected, string::joinStringsWithSeparator(input, "-"));
}

TEST_F(JoinStringsTests, EmptyItemsInVector)
{
  const std::vector<std::string> input{ "", "2", "", "", "5", "6", "" };
  const std::string expected{ "-2---5-6-" };
  ASSERT_EQ(expected, string::joinStringsWithSeparator(input, '-'));
  ASSERT_EQ(expected, string::joinStringsWithSeparator(input, "-"));
}

}  // namespace

}  // namespace test

}  // namespace utils

}  // namespace tijcore
