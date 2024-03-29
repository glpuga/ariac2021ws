/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// gtest
#include "gtest/gtest.h"

// tijcore
#include "mocks/OrderProcessingStrategyMock.hpp"

namespace tijcore
{
namespace test
{
namespace
{
using ::testing::Test;

class OrderProcessingStrategyInterfaceTests : public Test
{
};

TEST_F(OrderProcessingStrategyInterfaceTests, TheMockCanBeConstructed)
{
  // just testing that the mock builds
  OrderProcessingStrategyMock uut;
}

}  // namespace

}  // namespace test

}  // namespace tijcore
