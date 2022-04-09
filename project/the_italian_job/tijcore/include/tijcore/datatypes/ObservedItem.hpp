/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// tijcore
#include <tijcore/datatypes/AnonymizedDataHolder.hpp>
#include <tijcore/datatypes/QualifiedPartInfo.hpp>
#include <tijmath/RelativePose3.hpp>

namespace tijcore
{
struct ObservedItem
{
  // TODO(issue 132) this constructor is here to constrain the types of objects that can be used to
  // construct the item member, because the type-erase datatype allows for any type of storage.
  ObservedItem(const QualifiedPartInfo& pid, const tijmath::RelativePose3& rp)
    : item{ pid }, pose{ rp }
  {
  }

  ObservedItem(const ObservedItem&) = default;
  ObservedItem(ObservedItem&&) = default;
  ObservedItem& operator=(const ObservedItem&) = default;
  ObservedItem& operator=(ObservedItem&&) = default;

  /// type-erased data containing the observed item
  AnonymizedDataHolder item;

  /// pose of the observed item in an unknown reference frame
  tijmath::RelativePose3 pose;
};

}  // namespace tijcore
