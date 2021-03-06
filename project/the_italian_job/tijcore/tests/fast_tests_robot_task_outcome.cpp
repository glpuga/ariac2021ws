/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// Standard library
#include <iostream>
#include <sstream>

// gtest
#include "gtest/gtest.h"

// tijcore
#include <tijcore/agents/RobotTaskOutcome.hpp>

namespace tijcore {

namespace test {

namespace {

using ::testing::Test;

class RobotTaskOutcomeTests : public Test {
public:
};

TEST_F(RobotTaskOutcomeTests, TestFromString) {
  ASSERT_EQ(RobotTaskOutcome::TASK_SUCCESS,
            robot_task_outcome::fromString("TASK_SUCCESS"));
  ASSERT_EQ(RobotTaskOutcome::TASK_FAILURE,
            robot_task_outcome::fromString("TASK_FAILURE"));
  ASSERT_THROW(robot_task_outcome::fromString("bongo"), std::invalid_argument);
}

TEST_F(RobotTaskOutcomeTests, TestToString) {
  ASSERT_EQ("TASK_SUCCESS",
            robot_task_outcome::toString(RobotTaskOutcome::TASK_SUCCESS));
  ASSERT_EQ("TASK_FAILURE",
            robot_task_outcome::toString(RobotTaskOutcome::TASK_FAILURE));
}

TEST_F(RobotTaskOutcomeTests, TestStreamOperator) {
  std::ostringstream os;

  os << RobotTaskOutcome::TASK_SUCCESS << "\n"
     << RobotTaskOutcome::TASK_FAILURE << "\n";

  ASSERT_EQ("TASK_SUCCESS\n"
            "TASK_FAILURE\n",
            os.str());
}

} // namespace

} // namespace test

} // namespace tijcore
