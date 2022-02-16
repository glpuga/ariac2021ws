/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <string>

namespace tijcore
{
enum class RobotTaskOutcome
{
  TASK_SUCCESS,
  TASK_FAILURE
};

namespace robot_task_outcome
{
RobotTaskOutcome fromString(const std::string& sid);

std::string toString(const RobotTaskOutcome& id);

}  // namespace robot_task_outcome

std::ostream& operator<<(std::ostream& os, RobotTaskOutcome id);

}  // namespace tijcore
