/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <iostream>
#include <sstream>

// gtest
#include "gtest/gtest.h"

// tijmath
#include <tijmath/Position.hpp>

namespace tijmath
{
namespace test
{
namespace
{
using ::testing::Test;

class PositionTests : public Test
{
protected:
  const double position_tolerance_{ 1e-6 };
  const double increase_factor_{ 1.1 };
  const double decrease_factor_{ 0.9 };
};

TEST_F(PositionTests, DefaultConstructor)
{
  const Position uut = Position::fromVector(1, 2, 3);
  const auto dev_x = Vector3(position_tolerance_, 0, 0);
  const auto dev_y = Vector3(0, position_tolerance_, 0);
  const auto dev_z = Vector3(0, 0, position_tolerance_);

  ASSERT_TRUE(Position::samePosition(uut, uut, position_tolerance_));

  ASSERT_TRUE(Position::samePosition(uut, Position{ uut.vector() + decrease_factor_ * dev_x },
                                     position_tolerance_));
  ASSERT_TRUE(Position::samePosition(uut, Position{ uut.vector() + decrease_factor_ * dev_y },
                                     position_tolerance_));
  ASSERT_TRUE(Position::samePosition(uut, Position{ uut.vector() + decrease_factor_ * dev_z },
                                     position_tolerance_));
  ASSERT_TRUE(Position::samePosition(uut, Position{ uut.vector() - decrease_factor_ * dev_x },
                                     position_tolerance_));
  ASSERT_TRUE(Position::samePosition(uut, Position{ uut.vector() - decrease_factor_ * dev_y },
                                     position_tolerance_));
  ASSERT_TRUE(Position::samePosition(uut, Position{ uut.vector() - decrease_factor_ * dev_z },
                                     position_tolerance_));

  ASSERT_FALSE(Position::samePosition(uut, Position{ uut.vector() + increase_factor_ * dev_x },
                                      position_tolerance_));
  ASSERT_FALSE(Position::samePosition(uut, Position{ uut.vector() + increase_factor_ * dev_y },
                                      position_tolerance_));
  ASSERT_FALSE(Position::samePosition(uut, Position{ uut.vector() + increase_factor_ * dev_z },
                                      position_tolerance_));
  ASSERT_FALSE(Position::samePosition(uut, Position{ uut.vector() - increase_factor_ * dev_x },
                                      position_tolerance_));
  ASSERT_FALSE(Position::samePosition(uut, Position{ uut.vector() - increase_factor_ * dev_y },
                                      position_tolerance_));
  ASSERT_FALSE(Position::samePosition(uut, Position{ uut.vector() - increase_factor_ * dev_z },
                                      position_tolerance_));
}

}  // namespace

}  // namespace test

}  // namespace tijmath
