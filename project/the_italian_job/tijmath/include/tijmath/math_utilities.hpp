/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

namespace tijmath
{
namespace utils
{
namespace angles
{
constexpr double pi()
{
  return 3.14159265358979323846;
}

constexpr double degreesToRadians(const double deg)
{
  return deg * pi() / 180.0;
}

}  // namespace angles

}  // namespace utils

}  // namespace tijmath
