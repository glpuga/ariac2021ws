/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// gtest
#include "gtest/gtest.h"

// tijcore
#include "mocks/RobotActuatorsMock.hpp"

namespace tijcore
{
namespace test
{
namespace
{
using ::testing::Test;

class RobotActuatorsMockTests : public Test
{
public:
};

TEST_F(RobotActuatorsMockTests, TheMockCanBeConstructed)
{
  // just testing that the mock builds
  RobotActuatorsMock uut;
}

}  // namespace

}  // namespace test

}  // namespace tijcore
