/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <string>

namespace tijcore
{
enum class AgvId
{
  agv1,
  agv2,
  agv3,
  agv4,
  any
};

namespace agv
{
AgvId fromString(const std::string& sid);

std::string toString(const AgvId& id);

bool isAny(const AgvId& sid);

bool isValid(const std::string& sid);

}  // namespace agv

std::ostream& operator<<(std::ostream& os, AgvId id);

}  // namespace tijcore
