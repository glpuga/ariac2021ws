/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// TODO(glpuga) this file may not be needed in the end, depending on how
// injection is performed in TaskMaster

// standard library
#include <memory>

// gtest
#include "gmock/gmock.h"

// tijcore
#include <tijcore/perception/OrderProcessingStrategyFactoryInterface.hpp>

namespace tijcore {

class OrderProcessingStrategyFactoryMock
    : public OrderProcessingStrategyFactoryInterface {
public:
  using Ptr = std::unique_ptr<OrderProcessingStrategyFactoryMock>;

  MOCK_CONST_METHOD0(create, OrderProcessingStrategyInterface::Ptr());
};

} // namespace tijcore
