/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library
#include <ostream>
#include <stdexcept>
#include <string>
#include <unordered_map>

// tijcore
#include <tijcore/datatypes/PartColorId.hpp>

namespace tijcore
{
namespace part_color
{
PartColorId fromString(const std::string& sid)
{
  static std::unordered_map<std::string, PartColorId> id_map = {
    { "red", PartColorId::red },
    { "green", PartColorId::green },
    { "blue", PartColorId::blue },
  };

  auto it = id_map.find(sid);
  if (it == id_map.end())
  {
    throw std::invalid_argument{ "Invalid string part color id: " + sid };
  }

  return it->second;
}

std::string toString(const PartColorId& id)
{
  static const std::unordered_map<PartColorId, std::string> id_map = {
    { PartColorId::red, "red" },
    { PartColorId::green, "green" },
    { PartColorId::blue, "blue" },
  };

  auto it = id_map.find(id);
  if (it == id_map.end())
  {
    throw std::invalid_argument{ "Invalid part color id" };
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

};  // namespace part_color

std::ostream& operator<<(std::ostream& os, PartColorId id)
{
  return os << part_color::toString(id);
}

}  // namespace tijcore
