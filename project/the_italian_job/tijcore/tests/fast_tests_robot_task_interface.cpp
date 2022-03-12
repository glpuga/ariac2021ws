/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// gtest
#include "gtest/gtest.h"

// tijcore
#include "mocks/RobotTaskMock.hpp"

namespace tijcore
{
namespace test
{
namespace
{
using ::testing::Test;

class RobotTaskInterfaceTests : public Test
{
public:
};

TEST_F(RobotTaskInterfaceTests, TheMockCanBeConstructed)
{
  // just testing that the mock builds
  RobotTaskMock uut;
}

}  // namespace

}  // namespace test

}  // namespace tijcore
