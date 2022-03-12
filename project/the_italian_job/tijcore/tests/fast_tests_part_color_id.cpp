/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <iostream>
#include <sstream>

// gtest
#include "gtest/gtest.h"

// tijcore
#include <tijcore/datatypes/PartColorId.hpp>

namespace tijcore
{
namespace test
{
namespace
{
using ::testing::Test;

class PartColorTests : public Test
{
public:
};

TEST_F(PartColorTests, TestFromString)
{
  ASSERT_EQ(PartColorId::red, part_color::fromString("red"));
  ASSERT_EQ(PartColorId::green, part_color::fromString("green"));
  ASSERT_EQ(PartColorId::blue, part_color::fromString("blue"));
  ASSERT_THROW(part_color::fromString("bongo"), std::invalid_argument);
}

TEST_F(PartColorTests, TestToString)
{
  ASSERT_EQ("red", part_color::toString(PartColorId::red));
  ASSERT_EQ("green", part_color::toString(PartColorId::green));
  ASSERT_EQ("blue", part_color::toString(PartColorId::blue));
}

TEST_F(PartColorTests, TestStreamOperator)
{
  std::ostringstream os;

  os << PartColorId::red << "\n" << PartColorId::green << "\n" << PartColorId::blue << "\n";

  ASSERT_EQ(
      "red\n"
      "green\n"
      "blue\n",
      os.str());
}

TEST_F(PartColorTests, TestIsValid)
{
  ASSERT_TRUE(part_color::isValid("red"));
  ASSERT_TRUE(part_color::isValid("green"));
  ASSERT_TRUE(part_color::isValid("blue"));

  ASSERT_FALSE(part_color::isValid(""));
  ASSERT_FALSE(part_color::isValid("white"));
  ASSERT_FALSE(part_color::isValid("black"));
  ASSERT_FALSE(part_color::isValid("foo"));
}

}  // namespace

}  // namespace test

}  // namespace tijcore
