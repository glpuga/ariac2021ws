/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library
#include <ostream>
#include <stdexcept>
#include <string>
#include <unordered_map>

// tijcore
#include <tijcore/datatypes/AgvId.hpp>

namespace tijcore
{
namespace agv
{
AgvId fromString(const std::string& sid)
{
  static std::unordered_map<std::string, AgvId> id_map = {
    { "agv1", AgvId::agv1 }, { "agv2", AgvId::agv2 }, { "agv3", AgvId::agv3 },
    { "agv4", AgvId::agv4 }, { "any", AgvId::any },
  };

  auto it = id_map.find(sid);
  if (it == id_map.end())
  {
    throw std::invalid_argument{ "Invalid string AGV id: " + sid };
  }

  return it->second;
}

std::string toString(const AgvId& id)
{
  static const std::unordered_map<AgvId, std::string> id_map = {
    { AgvId::agv1, "agv1" }, { AgvId::agv2, "agv2" }, { AgvId::agv3, "agv3" },
    { AgvId::agv4, "agv4" }, { AgvId::any, "any" },
  };

  auto it = id_map.find(id);
  if (it == id_map.end())
  {
    throw std::invalid_argument{ "Invalid AGV id" };
  }

  return it->second;
}

bool isAny(const AgvId& sid)
{
  return (sid == AgvId::any);
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

}  // namespace agv

std::ostream& operator<<(std::ostream& os, AgvId id)
{
  return os << agv::toString(id);
}

}  // namespace tijcore
