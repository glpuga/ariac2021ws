/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>
#include <string>

// tijcore
#include <tijcore/datatypes/AnonymizedDataHolder.hpp>
#include <tijcore/datatypes/QualifiedMovableTrayInfo.hpp>
#include <tijcore/datatypes/QualifiedPartInfo.hpp>
#include <tijmath/RelativePose3.hpp>
#include <tijutils/UniqueId.hpp>

namespace tijcore
{
class ManagedLocus : public std::enable_shared_from_this<ManagedLocus>
{
public:
  using SharedPtr = std::shared_ptr<ManagedLocus>;

  static void TransferPartFromHereToThere(ManagedLocus& here, ManagedLocus& there);

  static ManagedLocus CreateEmptyLocus(     //
      const std::string& parent_container,  //
      const tijmath::RelativePose3& pose);  //

  static ManagedLocus CreatePartLocus(      //
      const std::string& parent_container,  //
      const tijmath::RelativePose3& pose,   //
      const PartId& part_id,                //
      const bool broken);                   //

  static ManagedLocus CreateMovableTrayLocus(  //
      const std::string& parent_container,     //
      const tijmath::RelativePose3& pose,      //
      const MovableTrayId& movable_tray_id);   //

  std::string parentName() const;

  tijutils::UniqueId uniqueId() const;

  const tijmath::RelativePose3& pose() const;

  bool isEmptyLocus() const;

  bool isLocusWithPart() const;

  bool isLocusWithMovableTray() const;

  const QualifiedPartInfo& qualifiedPartInfo() const;

  const QualifiedMovableTrayInfo& qualifiedMovableTrayInfo() const;

  void resetUniqueId(const tijutils::UniqueId& new_unique_id);

private:
  struct QualifiedEmptyLocusInfo
  {
  };

  ManagedLocus(const std::string& parent_container, const tijmath::RelativePose3& pose,
               const AnonymizedDataHolder& locus_contents);

  std::string parent_container_;
  tijmath::RelativePose3 pose_;
  AnonymizedDataHolder locus_contents_;

  tijutils::UniqueId unique_id_{ tijutils::UniqueId::CreateNewId() };
};

}  // namespace tijcore
