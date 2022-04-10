/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>
#include <string>
#include <vector>

// tijcore
#include <tijcore/resources/ModelTraySharedAccessSpaceDescription.hpp>
#include <tijmath/RelativePose3.hpp>

namespace tijcore
{
class SceneConfigReaderInterface
{
public:
  using Ptr = std::unique_ptr<SceneConfigReaderInterface>;
  using SharedPtr = std::shared_ptr<SceneConfigReaderInterface>;

  virtual ~SceneConfigReaderInterface() = default;

  struct DeviceData
  {
    std::string name;
    std::string frame_id;
    std::string shared_access_space_id;
  };

  struct ConveyorBeltData
  {
    std::string name;
    std::string container_frame_id;
    std::string surface_frame_id;
    std::string shared_access_space_id;
    double meters_per_second{ 0.0 };
  };

  struct BinData
  {
    std::string name;
    std::string frame_id;
    std::string shared_access_space_id;
  };

  virtual const std::string& getWorldFrameId() const = 0;

  virtual const tijmath::RelativePose3& getDropBucketPose() const = 0;

  virtual const std::vector<BinData>& getListOfBins() const = 0;

  virtual const std::vector<DeviceData>& getListOfLogicalCameras() const = 0;

  virtual const std::vector<DeviceData>& getListOfQualityControlSensors() const = 0;

  virtual const std::vector<DeviceData>& getListOfAgvs() const = 0;

  virtual const std::vector<DeviceData>& getListOfAssemblyStations() const = 0;

  virtual const std::vector<ConveyorBeltData>& getListOfConveyorBelts() const = 0;

  virtual const std::vector<ModelTraySharedAccessSpaceDescription>&
  getListOfSharedAccessSpaceDescriptions() const = 0;

  virtual const std::vector<tijmath::RelativePose3>& getListOfGantryPlanningHints() const = 0;

  virtual const std::vector<tijmath::RelativePose3>& getListOfSafeWaitingSpotHints() const = 0;
};

}  // namespace tijcore
