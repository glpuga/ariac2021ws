/* Copyright [2022] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>
#include <vector>

// third party
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/loggers/abstract_logger.h>

namespace tijbt
{
struct BTHandle
{
  std::unique_ptr<BT::Tree> tree;

  std::vector<std::unique_ptr<BT::StatusChangeLogger>> loggers;

  BTHandle() = default;
  // make sure handles cannot be copied
  BTHandle(BTHandle&&) = default;
  BTHandle& operator=(BTHandle&&) = default;
  BTHandle(const BTHandle&) = delete;
  BTHandle& operator=(const BTHandle&) = delete;
};

}  // namespace tijbt
