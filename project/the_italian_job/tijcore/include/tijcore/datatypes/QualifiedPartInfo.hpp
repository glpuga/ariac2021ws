/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// tijcore
#include <tijcore/datatypes/PartId.hpp>

namespace tijcore
{
struct QualifiedPartInfo
{
  PartId part_type{ PartId::UnkownPartId };
  bool part_is_broken{ false };
};

}  // namespace tijcore
