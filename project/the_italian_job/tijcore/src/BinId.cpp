/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <ostream>
#include <stdexcept>
#include <string>
#include <unordered_map>

// tijcore
#include <tijcore/datatypes/BinId.hpp>

namespace tijcore
{
namespace bin
{
BinId fromString(const std::string& sid)
{
  static std::unordered_map<std::string, BinId> id_map = {
    { "bin1", BinId::bin1 }, { "bin2", BinId::bin2 }, { "bin3", BinId::bin3 },
    { "bin4", BinId::bin4 }, { "bin5", BinId::bin5 }, { "bin6", BinId::bin6 },
    { "bin7", BinId::bin7 }, { "bin8", BinId::bin8 },
  };

  auto it = id_map.find(sid);
  if (it == id_map.end())
  {
    throw std::invalid_argument{ "Invalid string bin id: " + sid };
  }

  return it->second;
}

std::string toString(const BinId& id)
{
  static const std::unordered_map<BinId, std::string> id_map = {
    { BinId::bin1, "bin1" }, { BinId::bin2, "bin2" }, { BinId::bin3, "bin3" },
    { BinId::bin4, "bin4" }, { BinId::bin5, "bin5" }, { BinId::bin6, "bin6" },
    { BinId::bin7, "bin7" }, { BinId::bin8, "bin8" },
  };

  auto it = id_map.find(id);
  if (it == id_map.end())
  {
    throw std::invalid_argument{ "Invalid bin id" };
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

};  // namespace bin

std::ostream& operator<<(std::ostream& os, BinId id)
{
  return os << bin::toString(id);
}

}  // namespace tijcore
