/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// gtest
#include "gtest/gtest.h"

// tijcore
#include "mocks/RobotTaskGroupRunnerMock.hpp"

namespace tijcore
{
namespace test
{
namespace
{
using ::testing::Test;

class RobotTaskGroupRunnerMockTests : public Test
{
public:
};

TEST_F(RobotTaskGroupRunnerMockTests, TheMockCanBeConstructed)
{
  // just testing that the mock builds
  RobotTaskGroupRunnerMock uut;
}

}  // namespace

}  // namespace test

}  // namespace tijcore
