/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <string>

namespace tijcore
{
enum class MovableTrayId
{
  movable_tray_dark_wood,
  movable_tray_light_wood,
  movable_tray_metal_rusty,
  movable_tray_metal_shiny,
};

namespace movable_tray
{
MovableTrayId fromString(const std::string& sid);

std::string toString(const MovableTrayId& id);

bool isValid(const std::string& sid);

}  // namespace movable_tray

std::ostream& operator<<(std::ostream& os, MovableTrayId id);

}  // namespace tijcore
