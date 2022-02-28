/* Copyright [2022] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>
#include <vector>

// third party
#include <behaviortree_cpp_v3/behavior_tree.h>

// project
#include <tijbt/modular_loggers/ModularAbstractLogger.hpp>

namespace tijbt
{
struct BTHandle
{
  std::unique_ptr<BT::Tree> tree;

  std::vector<ModularStatusChangeLogger::Ptr> loggers;

  BTHandle() = default;
  // make sure handles cannot be copied
  BTHandle(BTHandle&&) = default;
  BTHandle& operator=(BTHandle&&) = default;
  BTHandle(const BTHandle&) = delete;
  BTHandle& operator=(const BTHandle&) = delete;
};

}  // namespace tijbt
