/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <string>

// external
#include "behaviortree_cpp_v3/behavior_tree.h"

// project
#include <tijcore/datatypes/GripperTypeId.hpp>

namespace BT
{
template <>
inline tijcore::GripperTypeId convertFromString(StringView str)
{
  const auto value = tijcore::gripper_type::fromString(str.data());
  return value;
}
}  // end namespace BT
