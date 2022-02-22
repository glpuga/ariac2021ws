/* Copyright [2022] <theItalianJob>
 * Author: Gerardo Puga
 */

// standard library
#include <memory>
#include <utility>

// project
#include <tijbt/BTExecutionResult.hpp>
#include <tijbt/BehaviorTreeManager.hpp>
#include <tijlogger/logger.hpp>

namespace tijbt
{
BehaviorTreeManager::BehaviorTreeManager(BTHandle bt_handle) : bt_handle_{ std::move(bt_handle) }
{
}

BehaviorTreeManager::~BehaviorTreeManager()
{
}

BTExecutionResult BehaviorTreeManager::run()
{
  BT::NodeStatus tree_status = BT::NodeStatus::RUNNING;

  try
  {
    halted_ = false;
    // fast track, if tickRoot() does not return RUNNING, we never sleep and terminate quickly
    tree_status = bt_handle_.tree->tickRoot();
    // slow track, if we enter the loop we wait a tick interval before calling back.
    while ((tree_status == BT::NodeStatus::RUNNING) && !halted_)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds{ 100 });
      tree_status = bt_handle_.tree->tickRoot();
    }
  }
  catch (const std::exception& e)
  {
    ERROR("Failure ticking the tree: {}", e.what());
    return BTExecutionResult::ERROR;
  }

  return tree_status == BT::NodeStatus::SUCCESS ? BTExecutionResult::SUCCESS :
                                                  BTExecutionResult::FAILURE;
}

void BehaviorTreeManager::halt()
{
  WARNING("Halting thread on user request");
  halted_ = true;
  bt_handle_.tree->haltTree();
}

}  // namespace tijbt
