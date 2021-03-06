/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <string>
#include <vector>

namespace tijcore {

namespace utils {

namespace angles {

constexpr double pi() { return 3.14159265358979323846; }

constexpr double degreesToRadians(const double deg) {
  return deg * pi() / 180.0;
}

} // namespace angles

} // namespace utils

} // namespace tijcore
