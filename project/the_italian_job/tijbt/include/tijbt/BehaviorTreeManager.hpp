/* Copyright [2022] <TheItalianJob>
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
class BehaviorTreeManager : public BehaviorTreeManagerInterface
{
public:
  explicit BehaviorTreeManager(BTHandle bt_handle);
  ~BehaviorTreeManager();

  BTExecutionResult run() override;

  void halt() override;

private:
  BTHandle bt_handle_;
};

}  // namespace tijbt
