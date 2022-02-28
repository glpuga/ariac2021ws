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
#include <tijbt/modular_loggers/ModularStdCoutLogger.hpp>
#include <tijbt/modular_loggers/ModularTIJBTLogger.hpp>
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

static const char* trivial_tree_failure = R"(
<root main_tree_to_execute = "MainTree" >
    <BehaviorTree ID="MainTree">
        <AlwaysSuccess/>
    </BehaviorTree>
</root> )";

static const char* operations_tree = R"(
<root main_tree_to_execute = "MainTree" >
    <BehaviorTree ID="MainTree">
        <SumNode op1="{operand_1}" op2="{operand_2}" out="{result}" />
    </BehaviorTree>

    <BehaviorTree ID="AlternativeTree">
        <DiffNode op1="{operand_1}" op2="{operand_2}" out="{result}" />
    </BehaviorTree>
</root> )";

static const char* mockable_tree = R"(
<root main_tree_to_execute = "MainTree" >
    <BehaviorTree ID="MainTree">
        <Sequence>
            <SubTree ID="Subtree1" />
            <SubTree ID="Subtree2" />
            <SubTree ID="Subtree3" />
        </Sequence>
    </BehaviorTree>

    <BehaviorTree ID="Subtree1">
        <AlwaysSuccess/>
    </BehaviorTree>

    <BehaviorTree ID="Subtree2">
        <AlwaysSuccess/>
    </BehaviorTree>

    <BehaviorTree ID="Subtree3">
        <AlwaysSuccess/>
    </BehaviorTree>
</root> )";

class SumNode : public BT::SyncActionNode
{
public:
  SumNode(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  BT::NodeStatus tick() override
  {
    auto op1 = getInput<double>("op1");
    auto op2 = getInput<double>("op2");
    if (!op1 || !op2)
    {
      throw BT::RuntimeError("missing required input for ", __PRETTY_FUNCTION__);
    }
    setOutput("out", op1.value() + op2.value());
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("op1"), BT::InputPort<double>("op2"),
             BT::OutputPort<double>("out") };
  }
};

class DiffNode : public BT::SyncActionNode
{
public:
  DiffNode(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  BT::NodeStatus tick() override
  {
    auto op1 = getInput<double>("op1");
    auto op2 = getInput<double>("op2");
    if (!op1 || !op2)
    {
      throw BT::RuntimeError("missing required input for ", __PRETTY_FUNCTION__);
    }
    setOutput("out", op1.value() - op2.value());
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("op1"), BT::InputPort<double>("op2"),
             BT::OutputPort<double>("out") };
  }
};

struct BehaviorTreeBuilderTest : public Test
{
  void SetUp() override
  {
    logger::instance().setLevel(logger::Level::Debug);
  }
};

TEST_F(BehaviorTreeBuilderTest, ConstructionTest)
{
  // the builder can be trivially constructed
  auto bt_factory_loader_function = [](BT::BehaviorTreeFactory&) {};
  BehaviorTreeBuilder uut(bt_factory_loader_function);
  (void)uut;
}

TEST_F(BehaviorTreeBuilderTest, CreationStepRequired)
{
  // calling createTree() to intantiate a builder instance is needed before calling anything else.
  auto bt_factory_loader_function = [](BT::BehaviorTreeFactory&) {};
  BehaviorTreeBuilder uut(bt_factory_loader_function);
  ASSERT_THROW({ uut.addBlackboard(BT::Blackboard::create()); }, std::runtime_error);
  ASSERT_THROW({ uut.addFileDescription("file_description"); }, std::runtime_error);
  ASSERT_THROW({ uut.addStringDescription("string_description"); }, std::runtime_error);
  ASSERT_THROW({ uut.addRootName("root_id"); }, std::runtime_error);
  ASSERT_THROW({ uut.addRootName("build"); }, std::runtime_error);
  ASSERT_THROW({ uut.addMockingFlag(); }, std::runtime_error);
}

TEST_F(BehaviorTreeBuilderTest, TrivialTreeConstructed)
{
  // a trivial tree can be constructed
  auto bt_factory_loader_function = [](BT::BehaviorTreeFactory&) {};

  BehaviorTreeBuilder uut(bt_factory_loader_function);
  ASSERT_NO_THROW({
    auto tree = uut.createTree()
                    .addStringDescription(trivial_tree_success)  //
                    .build();                                    //
    auto retval = tree->run();
    EXPECT_EQ(BTExecutionResult::SUCCESS, retval);
  });

  ASSERT_NO_THROW({
    auto tree = uut.createTree()
                    .addStringDescription(trivial_tree_failure)  //
                    .build();                                    //
    auto retval = tree->run();
    EXPECT_EQ(BTExecutionResult::SUCCESS, retval);
  });
}

TEST_F(BehaviorTreeBuilderTest, ThrowIfMultipleDescriptionsGiven)
{
  // throw an exception if multiple descriptions are given
  auto bt_factory_loader_function = [](BT::BehaviorTreeFactory&) {};

  BehaviorTreeBuilder uut(bt_factory_loader_function);

  ASSERT_THROW(
      {
        uut.createTree()
            .addStringDescription(trivial_tree_success)
            .addStringDescription(trivial_tree_success)
            .build();
      },

      std::runtime_error);
  ASSERT_THROW(
      {
        uut.createTree()
            .addStringDescription(trivial_tree_success)
            .addFileDescription("description_filename")
            .build();
      },
      std::runtime_error);

  ASSERT_THROW(
      {
        uut.createTree()
            .addFileDescription("description_filename")
            .addStringDescription(trivial_tree_success)
            .build();
      },
      std::runtime_error);

  ASSERT_THROW(
      {
        uut.createTree()
            .addFileDescription("description_filename")
            .addFileDescription("description_filename")
            .build();
      },
      std::runtime_error);
}

TEST_F(BehaviorTreeBuilderTest, TreeWithMultipleLoggers)
{
  // construct a tree with multiple loggers
  auto bt_factory_loader_function = [](BT::BehaviorTreeFactory&) {};

  BehaviorTreeBuilder uut(bt_factory_loader_function);
  auto tree = uut.createTree()
                  .addStringDescription(trivial_tree_success)           //
                  .addLogger(std::make_unique<ModularTIJBTLogger>())    //
                  .addLogger(std::make_unique<ModularStdCoutLogger>())  //
                  .addLogger(std::make_unique<ModularTIJBTLogger>())    //
                  .build();                                             //
  auto retval = tree->run();
  EXPECT_EQ(BTExecutionResult::SUCCESS, retval);
}

TEST_F(BehaviorTreeBuilderTest, UsingAndExternalBlackboard)
{
  // build a tree with an externa blackboard
  auto bt_factory_loader_function = [](BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<SumNode>("SumNode");
    factory.registerNodeType<DiffNode>("DiffNode");
  };

  BehaviorTreeBuilder uut(bt_factory_loader_function);
  {
    auto blackboard = BT::Blackboard::create();

    blackboard->set<double>("operand_1", 3.0);  // NOLINT(build/include_what_you_use)
    blackboard->set<double>("operand_2", 4.0);  // NOLINT(build/include_what_you_use)

    auto tree = uut.createTree()
                    .addStringDescription(operations_tree)              //
                    .addBlackboard(blackboard)                          //
                    .addLogger(std::make_unique<ModularTIJBTLogger>())  //
                    .build();                                           //

    auto retval = tree->run();

    EXPECT_EQ(BTExecutionResult::SUCCESS, retval);
    auto result = blackboard->get<double>("result");
    EXPECT_DOUBLE_EQ(7.0, result);
  }
}

TEST_F(BehaviorTreeBuilderTest, UsingAnAlternateRoot)
{
  // build a tree with an alternate root node
  auto bt_factory_loader_function = [](BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<SumNode>("SumNode");
    factory.registerNodeType<DiffNode>("DiffNode");
  };

  BehaviorTreeBuilder uut(bt_factory_loader_function);

  // regular root node (test baseline)
  {
    auto blackboard = BT::Blackboard::create();

    blackboard->set<double>("operand_1", 13.0);  // NOLINT(build/include_what_you_use)
    blackboard->set<double>("operand_2", 7.0);   // NOLINT(build/include_what_you_use)

    auto tree = uut.createTree()
                    .addStringDescription(operations_tree)              //
                    .addBlackboard(blackboard)                          //
                    .addLogger(std::make_unique<ModularTIJBTLogger>())  //
                    .build();                                           //

    auto retval = tree->run();

    EXPECT_EQ(BTExecutionResult::SUCCESS, retval);
    auto result = blackboard->get<double>("result");
    EXPECT_DOUBLE_EQ(20.0, result);
  }

  // alternative root node
  {
    auto blackboard = BT::Blackboard::create();

    blackboard->set<double>("operand_1", 13.0);  // NOLINT(build/include_what_you_use)
    blackboard->set<double>("operand_2", 7.0);   // NOLINT(build/include_what_you_use)

    auto tree = uut.createTree()
                    .addStringDescription(operations_tree)              //
                    .addBlackboard(blackboard)                          //
                    .addRootName("AlternativeTree")                     //
                    .addLogger(std::make_unique<ModularTIJBTLogger>())  //
                    .build();                                           //

    auto retval = tree->run();

    EXPECT_EQ(BTExecutionResult::SUCCESS, retval);
    auto result = blackboard->get<double>("result");
    EXPECT_DOUBLE_EQ(6.0, result);
  }
}

TEST_F(BehaviorTreeBuilderTest, SettingMockedBranches)
{
  // build a tree with mocked branches

  // baseline case: mocks are set in the bt factory, but not enabled through the flag,
  // and therefore they are not used
  {
    struct
    {
      MOCK_METHOD0(mockedRun, BT::NodeStatus());
    } node_mock;

    auto bt_factory_loader_function = [this, &node_mock](BT::BehaviorTreeFactory& factory) {
      BT::NodeBuilder mock_builder = [this, &node_mock](const std::string& name,
                                                        const BT::NodeConfiguration& config) {
        return std::make_unique<BT::SubtreeMockNode>(
            name, config,
            [&node_mock](BT::SubtreeMockBlackboardProxy&) { return node_mock.mockedRun(); });
      };
      factory.registerBuilder<BT::SubtreeMockNode>("SubtreeMock", mock_builder);
    };

    // on this run we don't expect the mock to be called ever
    EXPECT_CALL(node_mock, mockedRun()).Times(0);

    BehaviorTreeBuilder uut(bt_factory_loader_function);
    auto tree = uut.createTree().addStringDescription(mockable_tree).build();
    auto retval = tree->run();
    EXPECT_EQ(BTExecutionResult::SUCCESS, retval);
  }

  // test case: mocks are set in the factory and enable in the flag
  {
    struct
    {
      MOCK_METHOD0(mockedRun, BT::NodeStatus());
    } node_mock;

    auto bt_factory_loader_function = [this, &node_mock](BT::BehaviorTreeFactory& factory) {
      BT::NodeBuilder mock_builder = [this, &node_mock](const std::string& name,
                                                        const BT::NodeConfiguration& config) {
        return std::make_unique<BT::SubtreeMockNode>(
            name, config,
            [&node_mock](BT::SubtreeMockBlackboardProxy&) { return node_mock.mockedRun(); });
      };
      factory.registerBuilder<BT::SubtreeMockNode>("SubtreeMock", mock_builder);
    };

    // on this run we don't expect the mock to be called ever
    InSequence aux;
    EXPECT_CALL(node_mock, mockedRun()).WillOnce(Return(BT::NodeStatus::SUCCESS));
    EXPECT_CALL(node_mock, mockedRun()).WillOnce(Return(BT::NodeStatus::SUCCESS));
    EXPECT_CALL(node_mock, mockedRun()).WillOnce(Return(BT::NodeStatus::FAILURE));

    BehaviorTreeBuilder uut(bt_factory_loader_function);
    auto tree = uut.createTree()                                        //
                    .addStringDescription(mockable_tree)                //
                    .addMockingFlag()                                   //
                    .addLogger(std::make_unique<ModularTIJBTLogger>())  //
                    .build();
    auto retval = tree->run();
    EXPECT_EQ(BTExecutionResult::FAILURE, retval);
  }
}

}  // namespace

}  // namespace test

}  // namespace tijbt
