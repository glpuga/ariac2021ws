/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <string>

namespace tijcore
{
enum class GripperTypeId
{
  gripper_tray,
  gripper_type,
};

namespace gripper_type
{
GripperTypeId fromString(const std::string& sid);

std::string toString(const GripperTypeId& id);

bool isValid(const std::string& sid);

}  // namespace gripper_type

std::ostream& operator<<(std::ostream& os, GripperTypeId id);

}  // namespace tijcore
