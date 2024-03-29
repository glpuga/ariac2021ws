/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <string>

// project
#include <tijmath/Vector3.hpp>

namespace tijcore
{
enum class PartTypeId
{
  battery,
  sensor,
  regulator,
  pump
};

namespace part_type
{
PartTypeId fromString(const std::string& sid);

std::string toString(const PartTypeId& id);

bool isValid(const std::string& sid);

// TODO(glpuga) add tests for this function
tijmath::Vector3 dimensions(const PartTypeId& id);

}  // namespace part_type

std::ostream& operator<<(std::ostream& os, PartTypeId id);

}  // namespace tijcore
