/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>

// tijcore
#include <tijcore/agents/WorkRegionId.hpp>
#include <tijcore/localization/RelativePose3.hpp>
#include <tijcore/perception/ModelTraySharedAccessSpaceDescription.hpp>

namespace tijcore {

class SceneConfigReaderInterface {
public:
  using Ptr = std::unique_ptr<SceneConfigReaderInterface>;
  using SharedPtr = std::shared_ptr<SceneConfigReaderInterface>;

  virtual ~SceneConfigReaderInterface() = default;

  struct DeviceData {
    std::string name;
    std::string frame_id;
    std::string shared_access_space_id;
  };

  struct ConveyorBeltData {
    std::string name;
    std::string container_frame_id;
    std::string surface_frame_id;
    std::string shared_access_space_id;
    double meters_per_second{0.0};
  };

  struct BinData {
    std::string name;
    std::string frame_id;
    std::string shared_access_space_id;
    tijcore::WorkRegionId work_region;
  };

  virtual const std::string &getWorldFrameId() const = 0;

  virtual const tijcore::RelativePose3 &getDropBucketPose() const = 0;

  virtual const std::vector<BinData> &getListOfBins() const = 0;

  virtual const std::vector<DeviceData> &getListOfLogicalCameras() const = 0;

  virtual const std::vector<DeviceData> &
  getListOfQualityControlSensors() const = 0;

  virtual const std::vector<DeviceData> &getListOfAgvs() const = 0;

  virtual const std::vector<DeviceData> &getListOfAssemblyStations() const = 0;

  virtual const std::vector<ConveyorBeltData> &
  getListOfConveyorBelts() const = 0;

  virtual const std::vector<ModelTraySharedAccessSpaceDescription> &
  getListOfSharedAccessSpaceDescriptions() const = 0;

  virtual const std::vector<RelativePose3> &
  getListOfGantryPlanningHints() const = 0;

  virtual const std::vector<tijcore::RelativePose3> &
  getListOfSafeWaitingSpotHints() const = 0;
};

} // namespace tijcore
