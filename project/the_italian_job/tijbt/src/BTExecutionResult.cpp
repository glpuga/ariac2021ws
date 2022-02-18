/* Copyright [2022] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library
#include <ostream>
#include <stdexcept>
#include <string>
#include <unordered_map>

// tijcore
#include <tijbt/BTExecutionResult.hpp>

namespace tijbt
{
namespace behavior_tree_result
{
BTExecutionResult fromString(const std::string& sid)
{
  static std::unordered_map<std::string, BTExecutionResult> id_map = {
    { "SUCCESS", BTExecutionResult::SUCCESS },
    { "FAILURE", BTExecutionResult::FAILURE },
    { "ERROR", BTExecutionResult::ERROR },
  };

  auto it = id_map.find(sid);
  if (it == id_map.end())
  {
    throw std::invalid_argument{ "Invalid string BTExecutionResult id: " + sid };
  }

  return it->second;
}

std::string toString(const BTExecutionResult& id)
{
  static const std::unordered_map<BTExecutionResult, std::string> id_map = {
    { BTExecutionResult::SUCCESS, "SUCCESS" },
    { BTExecutionResult::FAILURE, "FAILURE" },
    { BTExecutionResult::ERROR, "ERROR" },
  };

  auto it = id_map.find(id);
  if (it == id_map.end())
  {
    throw std::invalid_argument{ "Invalid BTExecutionResult id" };
  }

  return it->second;
}

bool isValid(const std::string& sid)
{
  try
  {
    fromString(sid);
  }
  catch (const std::invalid_argument&)
  {
    return false;
  }
  return true;
}

}  // namespace behavior_tree_result

std::ostream& operator<<(std::ostream& os, BTExecutionResult id)
{
  return os << behavior_tree_result::toString(id);
}

}  // namespace tijbt
