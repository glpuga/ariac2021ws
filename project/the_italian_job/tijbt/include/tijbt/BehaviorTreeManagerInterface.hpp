/* Copyright [2022] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>

// project
#include <tijbt/BTExecutionResult.hpp>

namespace tijbt
{
class BehaviorTreeManagerInterface
{
public:
  using Ptr = std::unique_ptr<BehaviorTreeManagerInterface>;

  virtual ~BehaviorTreeManagerInterface() = default;

  virtual BTExecutionResult run() = 0;

  virtual void halt() = 0;
};

}  // namespace tijbt
