/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// TODO(glpuga) this file may not be needed in the end, depending on how
// injection is performed in TaskMaster

// standard library
#include <memory>

// tijcore
#include <tijcore/perception/OrderProcessingStrategyInterface.hpp>

namespace tijcore {

class OrderProcessingStrategyFactoryInterface {
public:
  using Ptr = std::unique_ptr<OrderProcessingStrategyFactoryInterface>;

  virtual OrderProcessingStrategyInterface::Ptr create() const = 0;
};

} // namespace tijcore
