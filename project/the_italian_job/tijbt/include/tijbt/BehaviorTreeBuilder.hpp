/* Copyright [2022] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <map>
#include <memory>
#include <string>
#include <vector>

// third party libraries
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/abstract_logger.h>

// project
#include <tijbt/BehaviorTreeBuilderInterface.hpp>

namespace tijbt
{
class BehaviorTreeBuilder : public BehaviorTreeBuilderInterface
{
public:
  using FactoryLoaderFunction = std::function<void(BT::BehaviorTreeFactory&)>;

  explicit BehaviorTreeBuilder(const FactoryLoaderFunction& factory_loader_function);

  BehaviorTreeBuilderInterface& createTree() override;

  BehaviorTreeBuilderInterface&
  addFileDescription(const std::string& description_file_full_path) override;

  BehaviorTreeBuilderInterface& addStringDescription(const std::string& description) override;

  BehaviorTreeBuilderInterface& addRootName(const std::string& root_name) override;

  BehaviorTreeBuilderInterface& addBlackboard(BT::Blackboard::Ptr blackboard) override;

  BehaviorTreeBuilderInterface& addMockingFlag() override;

  BehaviorTreeManagerInterface::Ptr build() override;

private:
  FactoryLoaderFunction factory_loader_function_;

  BehaviorTreeBuilderInterface::Ptr child_;

  bool builder_instance_{ false };

  std::string description_file_full_path_;
  std::string description_string_;

  std::string root_name_;
  BT::Blackboard::Ptr blackboard_;
  std::unique_ptr<BT::BehaviorTreeFactory> node_factory_;
  bool mocking_flag_{ false };

  BehaviorTreeBuilder(const FactoryLoaderFunction& factory_loader_function, const bool);

  void checkIfWeAreABuilderInstance() const;
};

}  // namespace tijbt
