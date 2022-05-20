/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <string>

// external
#include "behaviortree_cpp_v3/decorator_node.h"

// tijcore
#include <tijlogger/logger.hpp>

namespace tijcore
{
class TraceLoggerDecoratorNode : public BT::DecoratorNode
{
public:
  explicit TraceLoggerDecoratorNode(const std::string& name) : BT::DecoratorNode(name, {})
  {
    setRegistrationID("TraceLoggerDecorator");
  }

private:
  BT::NodeStatus tick() override
  {
    setStatus(BT::NodeStatus::RUNNING);

    if (inactive_)
    {
      inactive_ = false;
      WARNING("Entering branch {}", child_node_->name());
    }

    const BT::NodeStatus child_state = child_node_->executeTick();

    if (!inactive_ && child_state != BT::NodeStatus::RUNNING)
    {
      inactive_ = true;
      WARNING("Leaving branch {}", child_node_->name());
    }
    return child_state;
  }

private:
  bool inactive_{ true };
};
}  // namespace tijcore
