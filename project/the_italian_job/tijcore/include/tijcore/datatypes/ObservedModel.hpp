/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

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
