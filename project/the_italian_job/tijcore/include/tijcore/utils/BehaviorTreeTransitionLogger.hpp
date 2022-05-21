/* Copyright [2022] <TheItalianJob>
 * Author: Gerardo Puga
 *
 * Based on the CoutLogger in BehaviorTree.CPP, by Davide Faconti
 */

#pragma once

// standard library
#include <memory>

// bt
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

// project
#include <behavior_tree_extras/modular_loggers/ModularAbstractLogger.hpp>

namespace tijcore
{
/**
 * @brief Injectable StdCoutLogger logger that can be attached to a tree after creation. Allows
 *        dependency injection-ish mode of adding loggers to trees.
 */
class BehaviorTreeTransitionLogger : public behavior_tree_extras::ModularStatusChangeLogger
{
public:
  /** @brief Attaches the logger to a tree and complete initialization. Needs to be called before
   *         calling any other member function. */
  void attach(BT::Tree& tree) override;

  /** @brief Flushes the logger output. Behavior undefined if the logger has not been initialized
   *         before calling.*/
  void flush() override;

private:
  std::unique_ptr<BT::StatusChangeLogger> logger_;
};

}  // namespace tijcore
