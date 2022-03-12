/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// gtest
#include "gtest/gtest.h"

// tijcore
#include "mocks/TaskMasterMock.hpp"

namespace tijcore
{
namespace test
{
namespace
{
using ::testing::Test;

class TaskMasterInterfaceTests : public Test
{
public:
};

TEST_F(TaskMasterInterfaceTests, TheMockCanBeConstructed)
{
  // just testing that the mock builds
  TaskMasterMock uut;
}

}  // namespace

}  // namespace test

}  // namespace tijcore
