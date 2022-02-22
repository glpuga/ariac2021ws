/* Copyright [2022] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library
#include <memory>
#include <string>

// gtest
#include "gmock/gmock.h"
#include "gtest/gtest.h"

// third-party
#include <behaviortree_cpp_v3/decorators/subtreemock_node.h>

// project
#include <tijbt/BehaviorTreeBuilder.hpp>
#include <tijlogger/logger.hpp>

namespace tijbt
{
namespace test
{
namespace
{
using ::testing::InSequence;
using ::testing::Return;
using ::testing::Test;

static const char* trivial_tree_success = R"(
<root main_tree_to_execute = "MainTree" >
    <BehaviorTree ID="MainTree">
        <AlwaysSuccess/>
    </BehaviorTree>
</root> )";

static const char* mockable_tree = R"(
<root main_tree_to_execute = "MainTree" >
    <BehaviorTree ID="MainTree">
        <SubTree ID="MockMeHere" />
    </BehaviorTree>
</root> )";

static const char* pure_wait_tree = R"(
<root main_tree_to_execute = "MainTree" >
    <BehaviorTree ID="MainTree">
        <Delay delay_msec="2000">
            <AlwaysSuccess/>
        </Delay>
    </BehaviorTree>
</root> )";

class BehaviorTreeManagerTests : public Test
{
protected:
  std::chrono::milliseconds tick_interval_{ 100 };

  void SetUp() override
  {
    logger::instance().setLevel(logger::Level::Debug);
  }
};

TEST_F(BehaviorTreeManagerTests, ConstructionTest)
{
  // just construts a trivial tree and destroys without having ever used it
  auto bt_factory_loader_function = [](BT::BehaviorTreeFactory&) {};
  BehaviorTreeBuilder uut(bt_factory_loader_function);
  auto tree = uut.createTree()
                  .addStringDescription(trivial_tree_success)  //
                  .build();                                    //
}

TEST_F(BehaviorTreeManagerTests, NoUnnecessaryWaitingTimeIfNodesDontBlock)
{
  // tests that no redundant waiting time is introduced for trees with nodes that never return
  // RUNNING (e.g. only condition nodes)
  auto bt_factory_loader_function = [](BT::BehaviorTreeFactory&) {};
  BehaviorTreeBuilder uut(bt_factory_loader_function);
  auto tree = uut.createTree()
                  .addStringDescription(trivial_tree_success)  //
                  .build();                                    //

  const auto start_time = std::chrono::system_clock::now();
  auto return_value = tree->run();
  const auto end_time = std::chrono::system_clock::now();
  const auto run_time = end_time - start_time;

  ASSERT_EQ(BTExecutionResult::SUCCESS, return_value);
  ASSERT_GE(tick_interval_ / 4, run_time);
}

TEST_F(BehaviorTreeManagerTests, RunTimeMatchesTimesRootReturnsRUNNING)
{
  // test that if the root returns N times RUNNING, the runtime for the run method is
  // N * tick_interval_
  struct
  {
    MOCK_METHOD0(mockedTickFunction, BT::NodeStatus());
  } node_mock;

  auto bt_factory_loader_function = [this, &node_mock](BT::BehaviorTreeFactory& factory) {
    BT::NodeBuilder mock_builder = [this, &node_mock](const std::string& name,
                                                      const BT::NodeConfiguration& config) {
      return std::make_unique<BT::SubtreeMockNode>(
          name, config,
          [&node_mock](BT::SubtreeMockBlackboardProxy&) { return node_mock.mockedTickFunction(); });
    };
    factory.registerBuilder<BT::SubtreeMockNode>("SubtreeMock", mock_builder);
  };

  // test parameter
  const std::size_t N = 5;

  InSequence aux;
  EXPECT_CALL(node_mock, mockedTickFunction())
      .Times(N - 1)
      .WillRepeatedly(Return(BT::NodeStatus::RUNNING));
  EXPECT_CALL(node_mock, mockedTickFunction()).WillOnce(Return(BT::NodeStatus::FAILURE));

  BehaviorTreeBuilder uut(bt_factory_loader_function);
  auto tree = uut.createTree()                          //
                  .addStringDescription(mockable_tree)  //
                  .addMockingFlag()                     //
                  .build();

  const auto start_time = std::chrono::system_clock::now();
  auto return_value = tree->run();
  const auto end_time = std::chrono::system_clock::now();
  const auto run_time = end_time - start_time;

  ASSERT_EQ(BTExecutionResult::FAILURE, return_value);
  // the run time should be about N-1 tick intervals
  ASSERT_LE(tick_interval_ * (2 * N - 2) / 2, run_time);
  ASSERT_GE(tick_interval_ * (2 * N - 1) / 2, run_time);
}

TEST_F(BehaviorTreeManagerTests, HaltingATreeWorks)
{
  // tests that the execution of the tree can be stopped with halt()
  auto bt_factory_loader_function = [](BT::BehaviorTreeFactory&) {};
  BehaviorTreeBuilder uut(bt_factory_loader_function);

  const auto expected_runtime = std::chrono::seconds{ 2 };

  auto tree = uut.createTree()
                  .addStringDescription(pure_wait_tree)  //
                  .build();                              //

  const std::chrono::milliseconds thread_stopping_timestamp{ 200 };

  auto tree_halter_thread = [&tree, thread_stopping_timestamp]() {
    std::this_thread::sleep_for(thread_stopping_timestamp);
    tree->halt();
  };

  const auto start_time = std::chrono::system_clock::now();

  auto halter_thread_future = std::async(std::launch::async, tree_halter_thread);
  tree->run();
  const auto end_time = std::chrono::system_clock::now();

  // wait for the halter thread to terminate
  halter_thread_future.wait();

  const auto run_time = end_time - start_time;

  ASSERT_GE(thread_stopping_timestamp * 3 / 2, run_time);
}

}  // namespace

}  // namespace test

}  // namespace tijbt
