/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <string>

namespace tijcore
{
enum class MovableTrayTypeId
{
  movable_tray_dark_wood,
  movable_tray_light_wood,
  movable_tray_metal_rusty,
  movable_tray_metal_shiny,
};

namespace movable_tray
{
MovableTrayTypeId fromString(const std::string& sid);

std::string toString(const MovableTrayTypeId& id);

bool isValid(const std::string& sid);

}  // namespace movable_tray

std::ostream& operator<<(std::ostream& os, MovableTrayTypeId id);

}  // namespace tijcore
