/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>
#include <string>

namespace ek_challenger
{
class BehaviorTreeNode
{
public:
  BehaviorTreeNode();

  bool run();

private:
  const int32_t tick_rate_{ 10 };

  ros::NodeHandle nh_;

  std::unique_ptr<BT::Tree> tree_;
  std::unique_ptr<BT::StdCoutLogger> cout_logger_;
  std::unique_ptr<BT::PublisherZMQ> zmq_logger_;

  void registerNodes(BT::BehaviorTreeFactory& factory);
};

}  // namespace ek_challenger
