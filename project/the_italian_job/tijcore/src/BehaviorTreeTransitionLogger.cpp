
/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga
 *
 * Based on the CoutLogger in BehaviorTree.CPP, by Davide Faconti
 */

// Standard library
#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>

// external
#include "behaviortree_cpp_v3/loggers/abstract_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"

// project
#include <tijcore/utils/BehaviorTreeTransitionLogger.hpp>

namespace tijcore
{
class BehaviorTreeTransitionLoggerImpl : public BT::StatusChangeLogger
{
public:
  explicit BehaviorTreeTransitionLoggerImpl(const BT::Tree& tree)
    : StatusChangeLogger(tree.rootNode())
  {
  }

  void callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status,
                BT::NodeStatus status) override
  {
    constexpr const char* whitespaces = "                         ";
    constexpr const size_t ws_count = 25;
    double since_epoch = std::chrono::duration<double>(timestamp).count();
    printf("[%.3f]: %s%s %s -> %s", since_epoch, node.name().c_str(),
           &whitespaces[std::min(ws_count, node.name().size())],
           BT::toStr(prev_status, true).c_str(), toStr(status, true).c_str());
    std::cout << std::endl;
  }

  void flush() override
  {
    std::cout << std::flush;
  }
};

void BehaviorTreeTransitionLogger::flush()
{
  logger_->flush();
}

void BehaviorTreeTransitionLogger::attach(BT::Tree& tree)
{
  logger_ = std::make_unique<BehaviorTreeTransitionLoggerImpl>(tree);
}

}  // namespace tijcore
