/* Copyright [2022] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// third-party
#include <behaviortree_cpp_v3/loggers/abstract_logger.h>

namespace tijbt
{
class TIJBTLogger : public BT::StatusChangeLogger
{
public:
  explicit TIJBTLogger(const BT::Tree& tree);

  void callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status,
                BT::NodeStatus status) override;

  void flush() override;
};

}  // namespace tijbt
