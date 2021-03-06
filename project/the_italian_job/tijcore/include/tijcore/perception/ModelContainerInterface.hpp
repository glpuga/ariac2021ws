/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>
#include <string>

// tijcore
#include <tijcore/agents/WorkRegionId.hpp>
#include <tijcore/localization/RelativePose3.hpp>
#include <tijcore/math/CuboidVolume.hpp>
#include <tijcore/perception/ResourceHandle.hpp>

namespace tijcore {

class ModelContainerInterface {
public:
  using Ptr = std::unique_ptr<ModelContainerInterface>;
  using SharedPtr = std::shared_ptr<ModelContainerInterface>;

  ModelContainerInterface(const std::string &name,
                          const std::string &local_frame_id,
                          const RelativePose3 &pose,
                          const CuboidVolume &container_volume,
                          const std::string &shared_workspace_id);

  virtual ~ModelContainerInterface() = default;

  virtual bool enabled() const = 0;

  virtual bool isSubmissionTray() const = 0;

  virtual WorkRegionId region() const = 0;

  virtual void setEnabled(const bool state) = 0;

  //
  // methods with local implementation

  std::string name() const;

  std::string localFrameId() const;

  RelativePose3 pose() const;

  CuboidVolume containerVolume() const;

  std::string exclusionZoneId() const;

private:
  std::string name_;
  std::string local_frame_id_;
  RelativePose3 pose_;
  CuboidVolume container_volume_;
  std::string shared_workspace_id_;
};

} // namespace tijcore
