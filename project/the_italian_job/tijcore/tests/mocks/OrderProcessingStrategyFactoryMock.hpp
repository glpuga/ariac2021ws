/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// TODO(glpuga) this file may not be needed in the end, depending on how
// injection is performed in TaskDispatcher

// standard library
#include <memory>

// gtest
#include "gmock/gmock.h"

// tijcore
#include <tijcore/abstractions/OrderProcessingStrategyFactoryInterface.hpp>

namespace tijcore
{
class OrderProcessingStrategyFactoryMock : public OrderProcessingStrategyFactoryInterface
{
public:
  using Ptr = std::unique_ptr<OrderProcessingStrategyFactoryMock>;

  MOCK_CONST_METHOD0(create, OrderProcessingStrategyInterface::Ptr());
};

}  // namespace tijcore
