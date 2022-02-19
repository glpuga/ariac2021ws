/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library
#include <ostream>
#include <stdexcept>
#include <string>
#include <unordered_map>

// tijcore
#include <tijcore/datatypes/StationId.hpp>

namespace tijcore
{
namespace station_id
{
StationId fromString(const std::string& sid)
{
  static const std::unordered_map<std::string, StationId> id_map = {
    // kitting
    { "ks1", StationId::ks1 },
    { "ks2", StationId::ks2 },
    { "ks3", StationId::ks3 },
    { "ks4", StationId::ks4 },
    // assembly
    { "as1", StationId::as1 },
    { "as2", StationId::as2 },
    { "as3", StationId::as3 },
    { "as4", StationId::as4 },
    // any
    { "any", StationId::any },
  };

  auto it = id_map.find(sid);
  if (it == id_map.end())
  {
    throw std::invalid_argument{ "Invalid string station id: " + sid };
  }

  return it->second;
}

std::string toString(const StationId& id)
{
  static const std::unordered_map<StationId, std::string> id_map = {
    // kitting
    { StationId::ks1, "ks1" },
    { StationId::ks2, "ks2" },
    { StationId::ks3, "ks3" },
    { StationId::ks4, "ks4" },
    // assembly
    { StationId::as1, "as1" },
    { StationId::as2, "as2" },
    { StationId::as3, "as3" },
    { StationId::as4, "as4" },
    // any
    { StationId::any, "any" },
  };

  auto it = id_map.find(id);
  if (it == id_map.end())
  {
    throw std::invalid_argument{ "Invalid station id" };
  }

  return it->second;
}

bool isKittingStation(const StationId& id)
{
  return ((id == StationId::ks1) || (id == StationId::ks2) || (id == StationId::ks3) ||
          (id == StationId::ks4));
}

bool isAssemblyStation(const StationId& id)
{
  return ((id == StationId::as1) || (id == StationId::as2) || (id == StationId::as3) ||
          (id == StationId::as4));
}

bool isAny(const StationId& id)
{
  return (id == StationId::any);
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

};  // namespace station_id

std::ostream& operator<<(std::ostream& os, StationId id)
{
  return os << station_id::toString(id);
}

}  // namespace tijcore
