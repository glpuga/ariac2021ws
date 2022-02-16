/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library
#include <ostream>
#include <stdexcept>
#include <string>

// tijcore
#include <tijcore/datatypes/RobotTaskOutcome.hpp>

namespace tijcore
{
namespace robot_task_outcome
{
RobotTaskOutcome fromString(const std::string& sid)
{
  if (sid == "TASK_SUCCESS")
  {
    return RobotTaskOutcome::TASK_SUCCESS;
  }
  else if (sid != "TASK_FAILURE")
  {
    throw std::invalid_argument{ "Invalid robot task outcome id: " + sid };
  }
  return RobotTaskOutcome::TASK_FAILURE;
}

std::string toString(const RobotTaskOutcome& id)
{
  switch (id)
  {
    case RobotTaskOutcome::TASK_SUCCESS:
      return "TASK_SUCCESS";
    case RobotTaskOutcome::TASK_FAILURE:
      return "TASK_FAILURE";
  }
  return "";
}

};  // namespace robot_task_outcome

std::ostream& operator<<(std::ostream& os, RobotTaskOutcome id)
{
  return os << robot_task_outcome::toString(id);
}

}  // namespace tijcore
