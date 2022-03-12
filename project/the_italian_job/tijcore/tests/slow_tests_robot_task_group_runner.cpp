/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// gtest
#include "gtest/gtest.h"

// standard library
#include <chrono>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

// tijcore
#include <tijcore/tasking/RobotTaskAsyncRunner.hpp>
#include <tijcore/tasking/RobotTaskGroupRunner.hpp>

#include "mocks/RobotTaskMock.hpp"
#include "utils/ActionQueue.hpp"

using namespace std::literals;  // NOLINT(build/namespaces)

namespace tijcore
{
namespace test
{
namespace
{
using ::testing::InSequence;
using ::testing::Invoke;
using ::testing::Test;

class SequenceChecker
{
public:
  MOCK_CONST_METHOD1(mark, void(const int));
};

class GenericTask : public RobotTaskInterface
{
public:
  GenericTask(const std::chrono::milliseconds& pause, const int id,
              const RobotTaskOutcome& return_value, SequenceChecker& checker)
    : pause_{ pause }, id_{ id }, return_value_{ return_value }, checker_{ checker }
  {
  }

  RobotTaskOutcome run() override
  {
    std::this_thread::sleep_for(pause_);
    checker_.mark(id_);
    return return_value_;
  }

  void halt() override
  {
  }

private:
  std::chrono::milliseconds pause_;
  int id_;
  RobotTaskOutcome return_value_;

  SequenceChecker& checker_;
};

class RobotTaskGroupRunnerTests : public Test
{
protected:
  utils::ActionQueue action_queueu_;
};

TEST_F(RobotTaskGroupRunnerTests, TheObjectCanBeCreatedAndDestructed)
{
  RobotTaskGroupRunner uut;
}

TEST_F(RobotTaskGroupRunnerTests, RunnerCanExecuteASingleTask)
{
  SequenceChecker checker;
  RobotTaskGroupRunner uut;

  action_queueu_.queueTestActionQueue([&] {
    RobotTaskInterface::Ptr task =
        std::make_unique<GenericTask>(1s, 1, RobotTaskOutcome::TASK_SUCCESS, checker);
    uut.add(task);
  });

  action_queueu_.queueTestActionQueue([&] { std::this_thread::sleep_for(1s); });

  EXPECT_CALL(checker, mark(1)).Times(1);

  action_queueu_.queueTestActionQueue([&] { std::this_thread::sleep_for(500ms); });

  ASSERT_TRUE(action_queueu_.runTestActionQueue());
}

TEST_F(RobotTaskGroupRunnerTests, MultipleTasksCoStarted)
{
  InSequence aux;
  SequenceChecker checker;
  RobotTaskGroupRunner uut;

  action_queueu_.queueTestActionQueue([&] {
    RobotTaskInterface::Ptr task =
        std::make_unique<GenericTask>(1500ms, 3, RobotTaskOutcome::TASK_SUCCESS, checker);
    uut.add(task);
  });

  action_queueu_.queueTestActionQueue([&] {
    RobotTaskInterface::Ptr task =
        std::make_unique<GenericTask>(1000ms, 2, RobotTaskOutcome::TASK_SUCCESS, checker);
    uut.add(task);
  });

  action_queueu_.queueTestActionQueue([&] {
    RobotTaskInterface::Ptr task =
        std::make_unique<GenericTask>(500ms, 1, RobotTaskOutcome::TASK_SUCCESS, checker);
    uut.add(task);
  });

  action_queueu_.queueTestActionQueue([&] { std::this_thread::sleep_for(500ms); });

  EXPECT_CALL(checker, mark(1)).Times(1);

  action_queueu_.queueTestActionQueue([&] { std::this_thread::sleep_for(500ms); });

  EXPECT_CALL(checker, mark(2)).Times(1);

  action_queueu_.queueTestActionQueue([&] { std::this_thread::sleep_for(500ms); });

  EXPECT_CALL(checker, mark(3)).Times(1);

  action_queueu_.queueTestActionQueue([&] { std::this_thread::sleep_for(500ms); });

  ASSERT_TRUE(action_queueu_.runTestActionQueue());
}

TEST_F(RobotTaskGroupRunnerTests, MultipleTasksStaggeredInTime)
{
  InSequence aux;
  SequenceChecker checker;
  RobotTaskGroupRunner uut;

  action_queueu_.queueTestActionQueue([&] {
    RobotTaskInterface::Ptr task =
        std::make_unique<GenericTask>(3000ms, 3, RobotTaskOutcome::TASK_SUCCESS, checker);
    uut.add(task);
  });

  action_queueu_.queueTestActionQueue([&] {
    RobotTaskInterface::Ptr task =
        std::make_unique<GenericTask>(500ms, 1, RobotTaskOutcome::TASK_SUCCESS, checker);
    uut.add(task);
  });

  action_queueu_.queueTestActionQueue([&] { std::this_thread::sleep_for(500ms); });

  EXPECT_CALL(checker, mark(1)).Times(1);

  action_queueu_.queueTestActionQueue([&] { std::this_thread::sleep_for(500ms); });

  action_queueu_.queueTestActionQueue([&] {
    RobotTaskInterface::Ptr task =
        std::make_unique<GenericTask>(500ms, 2, RobotTaskOutcome::TASK_SUCCESS, checker);
    uut.add(task);
  });

  action_queueu_.queueTestActionQueue([&] { std::this_thread::sleep_for(500ms); });

  EXPECT_CALL(checker, mark(2)).Times(1);

  action_queueu_.queueTestActionQueue([&] { std::this_thread::sleep_for(1500ms); });

  EXPECT_CALL(checker, mark(3)).Times(1);

  action_queueu_.queueTestActionQueue([&] { std::this_thread::sleep_for(1000ms); });

  ASSERT_TRUE(action_queueu_.runTestActionQueue());
}

TEST_F(RobotTaskGroupRunnerTests, EarlyDestruction)
{
  SequenceChecker checker;
  RobotTaskGroupRunner uut;

  auto task = std::make_unique<RobotTaskMock>();

  action_queueu_.queueTestActionQueue([&] {
    // TODO(glpuga) this mess is because add gmock does not really work with
    // move only parameters. A better solution to this is needed.
    RobotTaskInterface ::Ptr tmp = std::move(task);
    uut.add(tmp);
  });

  EXPECT_CALL(*task, run()).WillOnce(Invoke([] {
    std::this_thread::sleep_for(1s);
    return RobotTaskOutcome::TASK_SUCCESS;
  }));

  action_queueu_.queueTestActionQueue([&] { std::this_thread::sleep_for(500ms); });

  EXPECT_CALL(*task, halt()).Times(1);

  ASSERT_TRUE(action_queueu_.runTestActionQueue());
}

TEST_F(RobotTaskGroupRunnerTests, NormalDestruction)
{
  SequenceChecker checker;
  RobotTaskGroupRunner uut;

  auto task = std::make_unique<RobotTaskMock>();

  action_queueu_.queueTestActionQueue([&] {
    // TODO(glpuga) this mess is because add gmock does not really work with
    // move only parameters. A better solution to this is needed.
    RobotTaskInterface::Ptr tmp = std::move(task);
    uut.add(tmp);
  });

  EXPECT_CALL(*task, run()).WillOnce(Invoke([] {
    std::this_thread::sleep_for(200ms);
    return RobotTaskOutcome::TASK_SUCCESS;
  }));

  action_queueu_.queueTestActionQueue([&] { std::this_thread::sleep_for(500ms); });

  EXPECT_CALL(*task, halt()).Times(0);

  ASSERT_TRUE(action_queueu_.runTestActionQueue());
}

}  // namespace

}  // namespace test

}  // namespace tijcore
