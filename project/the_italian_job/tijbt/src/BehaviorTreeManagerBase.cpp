/* Copyright [2022] <theItalianJob>
 * Author: Gerardo Puga
 */

// standard library
#include <memory>
#include <utility>

// project
#include <tijbt/BTExecutionResult.hpp>
#include <tijbt/BehaviorTreeManagerBase.hpp>

namespace tijbt
{
BehaviorTreeManagerBase::BehaviorTreeManagerBase(BTHandle bt_handle) : bt_handle_{ std::move(bt_handle) }
{
}

BehaviorTreeManagerBase::~BehaviorTreeManagerBase()
{
}

BTExecutionResult BehaviorTreeManagerBase::run()
{
  BT::NodeStatus tree_status = BT::NodeStatus::RUNNING;

  try
  {
    while (tree_status == BT::NodeStatus::RUNNING)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds{ 100 });
      tree_status = bt_handle_.tree->tickRoot();
    }
  }
  catch (const std::exception& e)
  {
    return BTExecutionResult::ERROR;
  }

  return tree_status == BT::NodeStatus::SUCCESS ? BTExecutionResult::SUCCESS : BTExecutionResult::FAILURE;
}

void BehaviorTreeManagerBase::halt()
{
  bt_handle_.tree->haltTree();
}

}  // namespace tijbt
