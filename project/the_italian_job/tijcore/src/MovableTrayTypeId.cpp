/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <ostream>
#include <stdexcept>
#include <string>
#include <unordered_map>

// tijcore
#include <tijcore/datatypes/MovableTrayTypeId.hpp>

namespace tijcore
{
namespace movable_tray
{
MovableTrayTypeId fromString(const std::string& sid)
{
  static std::unordered_map<std::string, MovableTrayTypeId> id_map = {
    { "movable_tray_dark_wood", MovableTrayTypeId::movable_tray_dark_wood },
    { "movable_tray_light_wood", MovableTrayTypeId::movable_tray_light_wood },
    { "movable_tray_metal_rusty", MovableTrayTypeId::movable_tray_metal_rusty },
    { "movable_tray_metal_shiny", MovableTrayTypeId::movable_tray_metal_shiny },
  };

  auto it = id_map.find(sid);
  if (it == id_map.end())
  {
    throw std::invalid_argument{ "Invalid string movable tray id: " + sid };
  }

  return it->second;
}

std::string toString(const MovableTrayTypeId& id)
{
  static const std::unordered_map<MovableTrayTypeId, std::string> id_map = {
    { MovableTrayTypeId::movable_tray_dark_wood, "movable_tray_dark_wood" },
    { MovableTrayTypeId::movable_tray_light_wood, "movable_tray_light_wood" },
    { MovableTrayTypeId::movable_tray_metal_rusty, "movable_tray_metal_rusty" },
    { MovableTrayTypeId::movable_tray_metal_shiny, "movable_tray_metal_shiny" },
  };

  auto it = id_map.find(id);
  if (it == id_map.end())
  {
    throw std::invalid_argument{ "Invalid movable tray id" };
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

};  // namespace movable_tray

std::ostream& operator<<(std::ostream& os, MovableTrayTypeId id)
{
  return os << movable_tray::toString(id);
}

}  // namespace tijcore
