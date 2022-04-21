/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// gtest
#include "gtest/gtest.h"

// tijcore
#include "mocks/TaskDispatcherMock.hpp"

namespace tijcore
{
namespace test
{
namespace
{
using ::testing::Test;

class TaskDispatcherInterfaceTests : public Test
{
public:
};

TEST_F(TaskDispatcherInterfaceTests, TheMockCanBeConstructed)
{
  // just testing that the mock builds
  TaskDispatcherMock uut;
}

}  // namespace

}  // namespace test

}  // namespace tijcore
