/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <ostream>
#include <stdexcept>
#include <string>
#include <unordered_map>

// tijcore
#include <tijcore/datatypes/GripperTypeId.hpp>

namespace tijcore
{
namespace gripper_type
{
GripperTypeId fromString(const std::string& sid)
{
  static std::unordered_map<std::string, GripperTypeId> id_map = {
    { "gripper_tray", GripperTypeId::gripper_tray },
    { "gripper_type", GripperTypeId::gripper_type },
  };

  auto it = id_map.find(sid);
  if (it == id_map.end())
  {
    throw std::invalid_argument{ "Invalid string Gripper Type id: " + sid };
  }

  return it->second;
}

std::string toString(const GripperTypeId& id)
{
  static const std::unordered_map<GripperTypeId, std::string> id_map = {
    { GripperTypeId::gripper_tray, "gripper_tray" },
    { GripperTypeId::gripper_type, "gripper_type" },
  };

  auto it = id_map.find(id);
  if (it == id_map.end())
  {
    throw std::invalid_argument{ "Invalid Gripper Type id" };
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

}  // namespace gripper_type

std::ostream& operator<<(std::ostream& os, GripperTypeId id)
{
  return os << gripper_type::toString(id);
}

}  // namespace tijcore
