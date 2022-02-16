/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>
#include <string>

// tijcore
#include <tijcore/datatypes/WorkRegionId.hpp>
#include <tijcore/resources/ResourceHandle.hpp>
#include <tijcore/utils/CuboidVolume.hpp>
#include <tijmath/RelativePose3.hpp>

namespace tijcore
{
class ModelContainerInterface
{
public:
  using Ptr = std::unique_ptr<ModelContainerInterface>;
  using SharedPtr = std::shared_ptr<ModelContainerInterface>;

  ModelContainerInterface(const std::string& name, const std::string& container_reference_frame_id_,
                          const std::string& surface_reference_frame_id_, const tijmath::RelativePose3& pose,
                          const CuboidVolume& container_volume, const std::string& shared_workspace_id);

  virtual ~ModelContainerInterface() = default;

  virtual bool enabled() const = 0;

  virtual bool isSubmissionTray() const = 0;

  virtual WorkRegionId region() const = 0;

  virtual void setEnabled(const bool state) = 0;

  //
  // methods with local implementation

  std::string name() const;

  std::string containerReferenceFrameId() const;

  std::string surfaceReferenceFrameId() const;

  tijmath::RelativePose3 pose() const;

  CuboidVolume containerVolume() const;

  std::string exclusionZoneId() const;

private:
  std::string name_;
  std::string container_reference_frame_id_;
  std::string surface_reference_frame_id_;
  tijmath::RelativePose3 pose_;
  CuboidVolume container_volume_;
  std::string shared_workspace_id_;
};

}  // namespace tijcore
