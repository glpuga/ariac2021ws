/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <string>

namespace tijcore {

enum class WorkRegionId { kitting, assembly, conveyor_belt };

namespace work_region {

WorkRegionId fromString(const std::string &sid);

std::string toString(const WorkRegionId &id);

bool isValid(const std::string &sid);

} // namespace work_region

std::ostream &operator<<(std::ostream &os, WorkRegionId id);

} // namespace tijcore
