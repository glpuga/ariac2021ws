/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library
#include <tuple>

// tijcore
#include <tijcore/perception/ManagedLocus.hpp>

namespace tijcore
{
int32_t ManagedLocus::unique_id_counter_;

ManagedLocus ManagedLocus::CreateEmptySpace(const std::string& parent_container, const RelativePose3& pose)
{
  // can't use make shared because constructor is private
  return ManagedLocus(parent_container, pose);
}

ManagedLocus ManagedLocus::CreateOccupiedSpace(const std::string& parent_container, const RelativePose3& pose,
                                               const PartId& part_id, const bool broken)
{
  // can't use make shared because constructor is private
  return ManagedLocus(parent_container, pose, part_id, broken);
}

void ManagedLocus::TransferPartFromHereToThere(ManagedLocus& here, ManagedLocus& there)
{
  if (!here.isModel())
  {
    throw std::logic_error{ "Can't move part from an empty space" };
  }
  if (!there.isEmpty())
  {
    throw std::logic_error{ "Can't move part to an occupied space" };
  }
  std::swap(here.part_id_, there.part_id_);
  std::swap(here.is_model_, there.is_model_);
  std::swap(here.is_broken_, there.is_broken_);
  // note that the difficulty attribute does not get swapped, because its tied
  // to the pose
}

ManagedLocus::ManagedLocus(const std::string& parent_container, const RelativePose3& pose)
  : parent_container_{ parent_container }
  , pose_{ pose }
  , part_id_{ PartId(PartTypeId::pump, PartColorId::red) }
  , is_model_{ false }
  , is_broken_{ false }
{
}

ManagedLocus::ManagedLocus(const std::string& parent_container, const RelativePose3& pose, const PartId& part_id,
                           const bool broken)
  : parent_container_{ parent_container }, pose_{ pose }, part_id_{ part_id }, is_model_{ true }, is_broken_{ broken }
{
}

bool ManagedLocus::isEmpty() const
{
  return !is_model_;
}

bool ManagedLocus::isModel() const
{
  return is_model_;
}

void ManagedLocus::setBrokenState(const bool is_broken)
{
  if (isEmpty())
  {
    throw std::logic_error{ "Tried to set broken property of empty space" };
  }
  is_broken_ = is_broken;
}

std::tuple<PartId, bool> ManagedLocus::model() const
{
  if (isEmpty())
  {
    throw std::logic_error{ "Requested part id from empty space" };
  }
  return std::make_tuple(part_id_, is_broken_);
}

const RelativePose3& ManagedLocus::pose() const
{
  return pose_;
}

RelativePose3& ManagedLocus::pose()
{
  return pose_;
}

std::string ManagedLocus::parentName() const
{
  return parent_container_;
}

}  // namespace tijcore
