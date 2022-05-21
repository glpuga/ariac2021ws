/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// gtest
#include "gtest/gtest.h"

// tijcore
#include "mocks/PickAndPlaceRobotMovementsMock.hpp"

namespace tijcore
{
namespace test
{
namespace
{
using ::testing::Test;

class PickAndPlaceRobotInterfaceTests : public Test
{
public:
};

TEST_F(PickAndPlaceRobotInterfaceTests, TheMockCanBeConstructed)
{
  // just testing that the mock builds
  PickAndPlaceRobotMovementsMock uut;
}

}  // namespace

}  // namespace test

}  // namespace tijcore
