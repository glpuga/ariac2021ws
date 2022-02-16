/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <string>

namespace tijcore
{
enum class PartColorId
{
  red,
  green,
  blue
};

namespace part_color
{
PartColorId fromString(const std::string& sid);

std::string toString(const PartColorId& id);

bool isValid(const std::string& sid);

}  // namespace part_color

std::ostream& operator<<(std::ostream& os, PartColorId id);

}  // namespace tijcore
