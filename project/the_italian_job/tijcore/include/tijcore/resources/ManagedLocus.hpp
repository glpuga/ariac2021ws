/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>
#include <string>

// tijcore
#include <tijcore/datatypes/AnonymizedDataHolder.hpp>
#include <tijcore/datatypes/PartId.hpp>
#include <tijmath/RelativePose3.hpp>
#include <tijutils/UniqueId.hpp>

namespace tijcore
{
class ModelContainerInterface;

class ManagedLocus : public std::enable_shared_from_this<ManagedLocus>
{
public:
  using SharedPtr = std::shared_ptr<ManagedLocus>;
  using ParentPtr = std::shared_ptr<ModelContainerInterface>;

  static void TransferPartFromHereToThere(ManagedLocus& here, ManagedLocus& there);

  static ManagedLocus CreateEmptySpace(     //
      const std::string& parent_container,  //
      const tijmath::RelativePose3& pose);  //

  static ManagedLocus CreateOccupiedSpace(  //
      const std::string& parent_container,  //
      const tijmath::RelativePose3& pose,   //
      const PartId& part_id,                //
      const bool broken);                   //

  bool isEmpty() const;

  bool isModel() const;

  void setBroken(const bool is_broken);

  PartId partId() const;

  bool broken() const;

  tijmath::RelativePose3& pose();

  const tijmath::RelativePose3& pose() const;

  std::string parentName() const;

  tijutils::UniqueId uniqueId() const;

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
