/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <iostream>
#include <sstream>

// gtest
#include "gtest/gtest.h"

// tijcore
#include <tijcore/datatypes/MovableTrayTypeId.hpp>

namespace tijcore
{
namespace test
{
namespace
{
using ::testing::Test;

struct MovableTrayTypeIdTests : public Test
{
};

TEST_F(MovableTrayTypeIdTests, TestFromString)
{
  ASSERT_EQ(MovableTrayTypeId::movable_tray_dark_wood,              //
            movable_tray::fromString("movable_tray_dark_wood"));    //
  ASSERT_EQ(MovableTrayTypeId::movable_tray_light_wood,             //
            movable_tray::fromString("movable_tray_light_wood"));   //
  ASSERT_EQ(MovableTrayTypeId::movable_tray_metal_rusty,            //
            movable_tray::fromString("movable_tray_metal_rusty"));  //
  ASSERT_EQ(MovableTrayTypeId::movable_tray_metal_shiny,            //
            movable_tray::fromString("movable_tray_metal_shiny"));  //
  ASSERT_THROW(movable_tray::fromString("bongo"), std::invalid_argument);
}

TEST_F(MovableTrayTypeIdTests, TestToString)
{
  ASSERT_EQ("movable_tray_dark_wood",
            movable_tray::toString(MovableTrayTypeId::movable_tray_dark_wood));
  ASSERT_EQ("movable_tray_light_wood",
            movable_tray::toString(MovableTrayTypeId::movable_tray_light_wood));
  ASSERT_EQ("movable_tray_metal_rusty",
            movable_tray::toString(MovableTrayTypeId::movable_tray_metal_rusty));
  ASSERT_EQ("movable_tray_metal_shiny",
            movable_tray::toString(MovableTrayTypeId::movable_tray_metal_shiny));
}

TEST_F(MovableTrayTypeIdTests, TestStreamOperator)
{
  std::ostringstream os;

  os << MovableTrayTypeId::movable_tray_dark_wood << "\n"
     << MovableTrayTypeId::movable_tray_light_wood << "\n"
     << MovableTrayTypeId::movable_tray_metal_rusty << "\n"
     << MovableTrayTypeId::movable_tray_metal_shiny << "\n";

  ASSERT_EQ(
      "movable_tray_dark_wood\n"
      "movable_tray_light_wood\n"
      "movable_tray_metal_rusty\n"
      "movable_tray_metal_shiny\n",
      os.str());
}

TEST_F(MovableTrayTypeIdTests, TestIsValid)
{
  ASSERT_TRUE(movable_tray::isValid("movable_tray_dark_wood"));
  ASSERT_TRUE(movable_tray::isValid("movable_tray_light_wood"));
  ASSERT_TRUE(movable_tray::isValid("movable_tray_metal_rusty"));
  ASSERT_TRUE(movable_tray::isValid("movable_tray_metal_shiny"));

  ASSERT_FALSE(movable_tray::isValid(""));
  ASSERT_FALSE(movable_tray::isValid("gear"));
  ASSERT_FALSE(movable_tray::isValid("assembly"));
  ASSERT_FALSE(movable_tray::isValid("foo"));
}

}  // namespace

}  // namespace test

}  // namespace tijcore
