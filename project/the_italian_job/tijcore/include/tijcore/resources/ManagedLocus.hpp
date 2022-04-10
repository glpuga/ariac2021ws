/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>
#include <string>

// tijcore
#include <tijcore/datatypes/PartId.hpp>
#include <tijmath/RelativePose3.hpp>

namespace tijcore
{
class ModelContainerInterface;

class ManagedLocus : public std::enable_shared_from_this<ManagedLocus>
{
public:
  using SharedPtr = std::shared_ptr<ManagedLocus>;
  using ParentPtr = std::shared_ptr<ModelContainerInterface>;

  static void TransferPartFromHereToThere(ManagedLocus& here, ManagedLocus& there);

  static ManagedLocus CreateEmptySpace(const std::string& parent_container,
                                       const tijmath::RelativePose3& pose);
  static ManagedLocus CreateOccupiedSpace(const std::string& parent_container,
                                          const tijmath::RelativePose3& pose, const PartId& part_id,
                                          const bool broken);

  bool isEmpty() const;
  bool isModel() const;

  void setBroken(const bool is_broken);

  PartId partId() const;
  bool broken() const;

  tijmath::RelativePose3& pose();
  const tijmath::RelativePose3& pose() const;

  std::string parentName() const;

  int32_t uniqueId() const
  {
    return unique_id_;
  }

private:
  ManagedLocus(const std::string& parent_container, const tijmath::RelativePose3& pose);
  ManagedLocus(const std::string& parent_container, const tijmath::RelativePose3& pose,
               const PartId& part_id, const bool broken);

  std::string parent_container_;

  tijmath::RelativePose3 pose_;
  PartId part_id_;
  bool is_model_;
  bool is_broken_;

  int32_t difficulty_{ 0 };
  int32_t unique_id_{ generateUniqueId() };

  static int32_t unique_id_counter_;

  static int32_t generateUniqueId()
  {
    return ++unique_id_counter_;
  }
};

}  // namespace tijcore
