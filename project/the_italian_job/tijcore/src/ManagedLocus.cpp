/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <string>
#include <utility>

// tijcore
#include <tijcore/datatypes/QualifiedPartInfo.hpp>
#include <tijcore/resources/ManagedLocus.hpp>

namespace tijcore
{
ManagedLocus ManagedLocus::CreateEmptySpace(const std::string& parent_container,
                                            const tijmath::RelativePose3& pose)
{
  // can't use make shared because constructor is private
  return ManagedLocus(parent_container, pose, AnonymizedDataHolder{ QualifiedEmptyLocusInfo{} });
}

ManagedLocus ManagedLocus::CreateOccupiedSpace(const std::string& parent_container,
                                               const tijmath::RelativePose3& pose,
                                               const PartId& part_id, const bool broken)
{
  // can't use make shared because constructor is private
  return ManagedLocus(parent_container, pose,
                      AnonymizedDataHolder{ QualifiedPartInfo{ part_id, broken } });
}

void ManagedLocus::TransferPartFromHereToThere(ManagedLocus& here, ManagedLocus& there)
{
  if (here.isEmpty())
  {
    throw std::logic_error{ "Can't move part from an empty space" };
  }
  else if (!there.isEmpty())
  {
    throw std::logic_error{ "Can't move part to an occupied space" };
  }
  AnonymizedDataHolder tmp{ std::move(there.locus_contents_) };
  there.locus_contents_ = std::move(here.locus_contents_);
  here.locus_contents_ = std::move(tmp);
}

ManagedLocus::ManagedLocus(const std::string& parent_container, const tijmath::RelativePose3& pose,
                           const AnonymizedDataHolder& locus_contents)
  : parent_container_{ parent_container }, pose_{ pose }, locus_contents_{ locus_contents }
{
}

bool ManagedLocus::isEmpty() const
{
  return locus_contents_.is<QualifiedEmptyLocusInfo>();
}

bool ManagedLocus::isModel() const
{
  return locus_contents_.is<QualifiedPartInfo>();
}

void ManagedLocus::setBroken(const bool is_broken)
{
  if (!isModel())
  {
    throw std::logic_error{ "Unable to set the broken status of not-a-part" };
  }
  locus_contents_.as<QualifiedPartInfo>().part_is_broken = is_broken;
}

PartId ManagedLocus::partId() const
{
  if (!isModel())
  {
    throw std::logic_error{ "Unable to retrieve the party type of not-a-part" };
  }
  return locus_contents_.as<QualifiedPartInfo>().part_type;
}

bool ManagedLocus::broken() const
{
  if (isEmpty())
  {
    throw std::logic_error{ "Requested broken status from empty space" };
  }
  return locus_contents_.as<QualifiedPartInfo>().part_is_broken;
}

const tijmath::RelativePose3& ManagedLocus::pose() const
{
  return pose_;
}

tijmath::RelativePose3& ManagedLocus::pose()
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

}  // namespace tijcore
