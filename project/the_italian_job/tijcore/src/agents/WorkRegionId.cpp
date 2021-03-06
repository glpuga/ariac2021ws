/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// Standard library
#include <ostream>
#include <stdexcept>
#include <unordered_map>

// tijcore
#include <tijcore/agents/WorkRegionId.hpp>

namespace tijcore {

namespace work_region {

WorkRegionId fromString(const std::string &sid) {
  static std::unordered_map<std::string, WorkRegionId> id_map = {
      {"kitting", WorkRegionId::kitting},
      {"assembly", WorkRegionId::assembly},
      {"conveyor_belt", WorkRegionId::conveyor_belt},
  };

  auto it = id_map.find(sid);
  if (it == id_map.end()) {
    throw std::invalid_argument{"Invalid string for WorkRegionId: " + sid};
  }

  return it->second;
}

std::string toString(const WorkRegionId &id) {
  static const std::unordered_map<WorkRegionId, std::string> id_map = {
      {WorkRegionId::kitting, "kitting"},
      {WorkRegionId::assembly, "assembly"},
      {WorkRegionId::conveyor_belt, "conveyor_belt"},
  };

  auto it = id_map.find(id);
  if (it == id_map.end()) {
    throw std::invalid_argument{"Invalid WorkRegionId id"};
  }

  return it->second;
}

bool isValid(const std::string &sid) {
  try {
    fromString(sid);
  } catch (const std::invalid_argument &) {
    return false;
  }
  return true;
}

} // namespace work_region

std::ostream &operator<<(std::ostream &os, WorkRegionId id) {
  return os << work_region::toString(id);
}

} // namespace tijcore
