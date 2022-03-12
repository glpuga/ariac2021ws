/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// tijcore
#include <tijcore/datatypes/PartId.hpp>
#include <tijmath/RelativePose3.hpp>

namespace tijcore
{
struct ObservedModel
{
  PartId type;

  tijmath::RelativePose3 pose;

  bool broken{ false };
};

}  // namespace tijcore
