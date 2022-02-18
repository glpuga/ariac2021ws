/* Copyright [2022] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>

// project
#include <tijbt/BTHandle.hpp>
#include <tijbt/BehaviorTreeManagerInterface.hpp>

namespace tijbt
{
class BehaviorTreeManagerBase : public BehaviorTreeManagerInterface
{
public:
  explicit BehaviorTreeManagerBase(BTHandle bt_handle);
  ~BehaviorTreeManagerBase();

  BTExecutionResult run() override;

  void halt() override;

private:
  BTHandle bt_handle_;
};

}  // namespace tijbt
