/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <stdexcept>
#include <string>
#include <utility>

// tijcore
#include <tijcore/datatypes/QualifiedMovableTrayInfo.hpp>
#include <tijcore/datatypes/QualifiedPartInfo.hpp>
#include <tijcore/resources/ManagedLocus.hpp>

namespace tijcore
{
ManagedLocus ManagedLocus::CreateEmptyLocus(const std::string& parent_container,
                                            const tijmath::RelativePose3& pose)
{
  // can't use make shared because constructor is private
  return ManagedLocus(parent_container, pose, AnonymizedDataHolder{ QualifiedEmptyLocusInfo{} });
}

ManagedLocus ManagedLocus::CreatePartLocus(const std::string& parent_container,
                                           const tijmath::RelativePose3& pose,
                                           const PartId& part_id, const bool broken)
{
  // can't use make shared because constructor is private
  return ManagedLocus(parent_container, pose,
                      AnonymizedDataHolder{ QualifiedPartInfo{ part_id, broken } });
}

ManagedLocus ManagedLocus::CreateMovableTrayLocus(const std::string& parent_container,
                                                  const tijmath::RelativePose3& pose,
                                                  const MovableTrayId& movable_tray_id)
{
  // can't use make shared because constructor is private
  return ManagedLocus(parent_container, pose,
                      AnonymizedDataHolder{ QualifiedMovableTrayInfo{ movable_tray_id } });
}

void ManagedLocus::TransferPartFromHereToThere(ManagedLocus& here, ManagedLocus& there)
{
  if (here.isEmptyLocus())
  {
    throw std::logic_error{ "Can't move part from an empty space" };
  }
  else if (!there.isEmptyLocus())
  {
    throw std::logic_error{ "Can't move part to an occupied space" };
  }
  AnonymizedDataHolder aux{ std::move(there.locus_contents_) };
  there.locus_contents_ = std::move(here.locus_contents_);
  here.locus_contents_ = std::move(aux);
}

ManagedLocus::ManagedLocus(const std::string& parent_container, const tijmath::RelativePose3& pose,
                           const AnonymizedDataHolder& locus_contents)
  : parent_container_{ parent_container }, pose_{ pose }, locus_contents_{ locus_contents }
{
}

bool ManagedLocus::isEmptyLocus() const
{
  return locus_contents_.is<QualifiedEmptyLocusInfo>();
}

bool ManagedLocus::isLocusWithPart() const
{
  return locus_contents_.is<QualifiedPartInfo>();
}

bool ManagedLocus::isLocusWithMovableTray() const
{
  return locus_contents_.is<QualifiedMovableTrayInfo>();
}

const QualifiedPartInfo& ManagedLocus::qualifiedPartInfo() const
{
  if (!isLocusWithPart())
  {
    throw std::logic_error{ "Unable to retrieve the qualified part info " };
  }
  return locus_contents_.as<QualifiedPartInfo>();
}

const QualifiedMovableTrayInfo& ManagedLocus::qualifiedMovableTrayInfo() const
{
  if (!isLocusWithMovableTray())
  {
    throw std::logic_error{ "Unable to retrieve the qualified movable tray info " };
  }
  return locus_contents_.as<QualifiedMovableTrayInfo>();
}

const tijmath::RelativePose3& ManagedLocus::pose() const
{
  return pose_;
}

std::string ManagedLocus::parentName() const
{
  return parent_container_;
}

tijutils::UniqueId ManagedLocus::uniqueId() const
{
  return unique_id_;
}

void ManagedLocus::resetUniqueId(const tijutils::UniqueId& new_unique_id)
{
  unique_id_ = new_unique_id;
}

}  // namespace tijcore
