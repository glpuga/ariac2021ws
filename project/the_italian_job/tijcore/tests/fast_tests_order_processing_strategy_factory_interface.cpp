/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// gtest
#include "gtest/gtest.h"

// tijcore
#include "mocks/OrderProcessingStrategyFactoryMock.hpp"

namespace tijcore
{
namespace test
{
namespace
{
using ::testing::Test;

class OrderProcessingStrategyFactoryInterfaceTests : public Test
{
};

TEST_F(OrderProcessingStrategyFactoryInterfaceTests, TheMockCanBeConstructed)
{
  // just testing that the mock builds
  OrderProcessingStrategyFactoryMock uut;
}

}  // namespace

}  // namespace test

}  // namespace tijcore
