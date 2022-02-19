/* Copyright [2022] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>
#include <string>

// gtest
#include "gmock/gmock.h"

// tijcore
#include <tijbt/BehaviorTreeBuilderInterface.hpp>

namespace tijbt
{
class BehaviorTreeBuilderMock : public BehaviorTreeBuilderInterface
{
public:
  BehaviorTreeBuilderInterface& createTree() override
  {
    return *this;
  }

  BehaviorTreeBuilderInterface&
  addFileDescription(const std::string& description_file_full_path) override
  {
    return *this;
  }

  BehaviorTreeBuilderInterface& addStringDescription(const std::string& description) override
  {
    return *this;
  }

  BehaviorTreeBuilderInterface& addRootName(const std::string& root_name) override
  {
    return *this;
  }

  BehaviorTreeBuilderInterface& addBlackboard(BT::Blackboard::Ptr blackboard) override
  {
    return *this;
  }

  BehaviorTreeBuilderInterface& addMockingFlag()
  {
    return *this;
  }

  MOCK_METHOD0(build, BehaviorTreeManagerInterface::Ptr());
};

}  // namespace tijbt
