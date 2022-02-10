/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <string>

// tijcore
#include <tijcore/math/CuboidVolume.hpp>
#include <tijmath/localization/RelativePose3.hpp>

namespace tijcore
{
struct ModelTraySharedAccessSpaceDescription
{
  std::string id;
  tijmath::RelativePose3 center;
  double x_size{ 0 };
  double y_size{ 0 };
  double z_size{ 0 };
};

}  // namespace tijcore
