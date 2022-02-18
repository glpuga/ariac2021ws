/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// project
#include <tijbt/BTExecutionResult.hpp>

namespace tijbt
{
class BehaviorTreeManagerInterface
{
public:
  virtual ~BehaviorTreeManagerInterface() = default;

  virtual BTExecutionResult run() = 0;

  virtual void halt() = 0;
};

}  // namespace tijbt
