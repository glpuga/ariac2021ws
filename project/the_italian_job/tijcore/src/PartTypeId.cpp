/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <ostream>
#include <stdexcept>
#include <string>
#include <unordered_map>

// tijcore
#include <tijcore/datatypes/PartTypeId.hpp>

namespace tijcore
{
namespace part_type
{
PartTypeId fromString(const std::string& sid)
{
  static std::unordered_map<std::string, PartTypeId> id_map = {
    { "battery", PartTypeId::battery },
    { "sensor", PartTypeId::sensor },
    { "regulator", PartTypeId::regulator },
    { "pump", PartTypeId::pump },
  };

  auto it = id_map.find(sid);
  if (it == id_map.end())
  {
    throw std::invalid_argument{ "Invalid string part type id: " + sid };
  }

  return it->second;
}

std::string toString(const PartTypeId& id)
{
  static const std::unordered_map<PartTypeId, std::string> id_map = {
    { PartTypeId::battery, "battery" },
    { PartTypeId::sensor, "sensor" },
    { PartTypeId::regulator, "regulator" },
    { PartTypeId::pump, "pump" },
  };

  auto it = id_map.find(id);
  if (it == id_map.end())
  {
    throw std::invalid_argument{ "Invalid part type id" };
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

tijmath::Vector3 dimensions(const PartTypeId& id)
{
  static const std::unordered_map<PartTypeId, tijmath::Vector3> id_map = {
    { PartTypeId::battery, tijmath::Vector3{ 0.136, 0.06, 0.05 } },
    { PartTypeId::sensor, tijmath::Vector3{ 0.125, 0.105, 0.06 } * 1.1 },
    { PartTypeId::regulator, tijmath::Vector3{ 0.093, 0.126, 0.066 } },
    { PartTypeId::pump, tijmath::Vector3{ 0.12, 0.12, 0.12 } },
  };
  auto it = id_map.find(id);
  if (it == id_map.end())
  {
    throw std::invalid_argument{ "Invalid part type id" };
  }

  return it->second;
}

};  // namespace part_type

std::ostream& operator<<(std::ostream& os, PartTypeId id)
{
  return os << part_type::toString(id);
}

}  // namespace tijcore
