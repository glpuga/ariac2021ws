/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <iostream>
#include <sstream>

// gtest
#include "gtest/gtest.h"

// tijcore
#include <tijcore/datatypes/GripperTypeId.hpp>

namespace tijcore
{
namespace test
{
namespace
{
using ::testing::Test;

struct GripperTypeIdTests : public Test
{
};

TEST_F(GripperTypeIdTests, TestFromString)
{
  ASSERT_EQ(GripperTypeId::gripper_tray, gripper_type::fromString("gripper_tray"));
  ASSERT_EQ(GripperTypeId::gripper_part, gripper_type::fromString("gripper_part"));
  ASSERT_THROW(gripper_type::fromString("agv5"), std::invalid_argument);
}

TEST_F(GripperTypeIdTests, TestToString)
{
  ASSERT_EQ("gripper_tray", gripper_type::toString(GripperTypeId::gripper_tray));
  ASSERT_EQ("gripper_part", gripper_type::toString(GripperTypeId::gripper_part));
}

TEST_F(GripperTypeIdTests, TestStreamOperator)
{
  std::ostringstream os;

  os << GripperTypeId::gripper_tray << "\n" << GripperTypeId::gripper_part << "\n";

  ASSERT_EQ(
      "gripper_tray\n"
      "gripper_part\n",
      os.str());
}

TEST_F(GripperTypeIdTests, TestIsValid)
{
  ASSERT_TRUE(gripper_type::isValid("gripper_tray"));
  ASSERT_TRUE(gripper_type::isValid("gripper_part"));

  ASSERT_FALSE(gripper_type::isValid(""));
  ASSERT_FALSE(gripper_type::isValid("agv0"));
  ASSERT_FALSE(gripper_type::isValid("agv5"));
  ASSERT_FALSE(gripper_type::isValid("foo"));
}

}  // namespace

}  // namespace test

}  // namespace tijcore
