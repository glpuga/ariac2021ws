/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// Standard library
#include <iostream>
#include <sstream>

// gtest
#include "gtest/gtest.h"

// tijcore
#include <tijcore/agents/OrderId.hpp>

namespace tijcore {

namespace test {

namespace {

using ::testing::Test;

class OrderIdTests : public Test {};

TEST_F(OrderIdTests, SimpleOrder) {
  OrderId uut{"order_1"};
  ASSERT_EQ(1, uut.id());
  ASSERT_EQ(false, uut.isUpdate());
  ASSERT_EQ("order_1", uut.codedString());
}

TEST_F(OrderIdTests, SimpleOrderUpdate) {
  OrderId uut{"order_99_update"};
  ASSERT_EQ(99, uut.id());
  ASSERT_EQ(true, uut.isUpdate());
  ASSERT_EQ("order_99_update", uut.codedString());
}

TEST_F(OrderIdTests, ConstructionFromFields) {
  {
    OrderId uut{66};
    ASSERT_EQ(66, uut.id());
    ASSERT_EQ(false, uut.isUpdate());
    ASSERT_EQ("order_66", uut.codedString());
  }
  {
    OrderId uut{77, false};
    ASSERT_EQ(77, uut.id());
    ASSERT_EQ(false, uut.isUpdate());
    ASSERT_EQ("order_77", uut.codedString());
  }
  {
    OrderId uut{88, true};
    ASSERT_EQ(88, uut.id());
    ASSERT_EQ(true, uut.isUpdate());
    ASSERT_EQ("order_88_update", uut.codedString());
  }
}

TEST_F(OrderIdTests, InvalidConstructionStrings) {
  EXPECT_THROW(OrderId uut(-1), std::invalid_argument);
  EXPECT_THROW(OrderId uut(-1, false), std::invalid_argument);
  EXPECT_THROW(OrderId uut(-1, true), std::invalid_argument);
  EXPECT_THROW(OrderId uut("order_update"), std::invalid_argument);
  EXPECT_THROW(OrderId uut("order_01_update"), std::invalid_argument);
  EXPECT_THROW(OrderId uut("foo"), std::invalid_argument);
  EXPECT_THROW(OrderId uut(""), std::invalid_argument);
  EXPECT_THROW(OrderId uut("order"), std::invalid_argument);
  EXPECT_THROW(OrderId uut("order_"), std::invalid_argument);
  EXPECT_THROW(OrderId uut("1"), std::invalid_argument);
  EXPECT_THROW(OrderId uut("1_update"), std::invalid_argument);
}

TEST_F(OrderIdTests, Equalities) {
  ASSERT_TRUE(OrderId("order_99") == OrderId("order_99"));
  ASSERT_TRUE(OrderId("order_99") == OrderId("order_99_update"));
  ASSERT_FALSE(OrderId("order_9") == OrderId("order_99"));
  ASSERT_FALSE(OrderId("order_9") == OrderId("order_99"));
}

} // namespace

} // namespace test

} // namespace tijcore
