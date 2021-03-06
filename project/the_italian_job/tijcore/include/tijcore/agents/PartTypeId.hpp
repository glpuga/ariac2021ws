/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <string>

namespace tijcore {

enum class PartTypeId { battery, sensor, regulator, pump };

namespace part_type {

PartTypeId fromString(const std::string &sid);

std::string toString(const PartTypeId &id);

bool isValid(const std::string &sid);

} // namespace part_type

std::ostream &operator<<(std::ostream &os, PartTypeId id);

} // namespace tijcore
