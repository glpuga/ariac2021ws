/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// tijcore
#include <tijcore/agents/PartId.hpp>
#include <tijcore/localization/RelativePose3.hpp>

namespace tijcore {

struct ObservedModel {
  PartId type;

  RelativePose3 pose;

  bool broken{false};
};

} // namespace tijcore
