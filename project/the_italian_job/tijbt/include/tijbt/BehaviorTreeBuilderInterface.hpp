/* Copyright [2022] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <functional>
#include <memory>
#include <string>

// third party libraries
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/abstract_logger.h>

// project
#include <tijbt/BehaviorTreeManagerInterface.hpp>
#include <tijbt/modular_loggers/ModularAbstractLogger.hpp>
namespace tijbt
{
class BehaviorTreeBuilderInterface
{
public:
  using Ptr = std::unique_ptr<BehaviorTreeBuilderInterface>;

  virtual ~BehaviorTreeBuilderInterface() = default;

  virtual BehaviorTreeBuilderInterface& createTree() = 0;

  virtual BehaviorTreeBuilderInterface&
  addFileDescription(const std::string& description_file_full_path) = 0;

  virtual BehaviorTreeBuilderInterface& addStringDescription(const std::string& description) = 0;

  virtual BehaviorTreeBuilderInterface& addRootName(const std::string& root_name) = 0;

  virtual BehaviorTreeBuilderInterface& addBlackboard(BT::Blackboard::Ptr blackboard) = 0;

  virtual BehaviorTreeBuilderInterface& addMockingFlag() = 0;

  virtual BehaviorTreeBuilderInterface& addLogger(ModularStatusChangeLogger::Ptr logger) = 0;

  virtual BehaviorTreeManagerInterface::Ptr build() = 0;
};

}  // namespace tijbt
