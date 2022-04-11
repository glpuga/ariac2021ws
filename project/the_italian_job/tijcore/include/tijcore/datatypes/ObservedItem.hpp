/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// tijcore
#include <tijcore/datatypes/AnonymizedDataHolder.hpp>
#include <tijcore/datatypes/QualifiedMovableTrayInfo.hpp>
#include <tijcore/datatypes/QualifiedPartInfo.hpp>
#include <tijmath/RelativePose3.hpp>
#include <tijutils/WallClockTimeStamp.hpp>

namespace tijcore
{
struct ObservedItem
{
  // TODO(issue 132) these constructors are here to constrain the types of objects that can be used
  // to construct the item member, because the type-erase datatype allows for any type of storage.
  ObservedItem(const QualifiedPartInfo& pid, const tijmath::RelativePose3& rp,
               const tijutils::WallClockTimeStamp& ts = std::chrono::system_clock::now())
    : item{ pid }, pose{ rp }, timestamp{ ts }
  {
  }

  ObservedItem(const QualifiedMovableTrayInfo& mtid, const tijmath::RelativePose3& rp,
               const tijutils::WallClockTimeStamp& ts = std::chrono::system_clock::now())
    : item{ mtid }, pose{ rp }, timestamp{ ts }
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

  /// pose of the observed item in an unknown reference frame
  tijutils::WallClockTimeStamp timestamp;
};

}  // namespace tijcore
