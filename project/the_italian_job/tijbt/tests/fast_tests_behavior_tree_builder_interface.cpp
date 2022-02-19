/* Copyright [2022] <TheItalianJob>
 * Author: Gerardo Puga
 */

// gtest
#include "gtest/gtest.h"

// tijcore
#include "mocks/BehaviorTreeBuilderMock.hpp"

namespace tijbt
{
namespace test
{
namespace
{
using ::testing::Test;

class BehaviorTreeBuilderInterfaceTests : public Test
{
};

TEST_F(BehaviorTreeBuilderInterfaceTests, TheMockCanBeConstructed)
{
  // just testing that the mock builds
  BehaviorTreeBuilderMock uut;
}

}  // namespace

}  // namespace test

}  // namespace tijbt
