/* Copyright [2022] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <string>

namespace tijbt
{
enum class BTExecutionResult
{
  SUCCESS,
  FAILURE,
  ERROR
};

namespace behavior_tree_result
{
BTExecutionResult fromString(const std::string& sid);

std::string toString(const BTExecutionResult& id);

bool isValid(const std::string& sid);

}  // namespace behavior_tree_result

std::ostream& operator<<(std::ostream& os, BTExecutionResult id);

}  // namespace tijbt
