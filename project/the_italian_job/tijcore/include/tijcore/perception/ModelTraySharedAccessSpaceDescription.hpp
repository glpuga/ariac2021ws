/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <string>

// tijcore
#include <tijcore/localization/RelativePose3.hpp>
#include <tijcore/math/CuboidVolume.hpp>

namespace tijcore
{
struct ModelTraySharedAccessSpaceDescription
{
  std::string id;
  RelativePose3 center;
  double x_size{ 0 };
  double y_size{ 0 };
  double z_size{ 0 };
};

}  // namespace tijcore
