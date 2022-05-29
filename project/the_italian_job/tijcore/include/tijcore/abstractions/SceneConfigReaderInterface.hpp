/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>
#include <string>
#include <vector>

// tijcore
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
  };

  struct ConveyorBeltData
  {
    std::string name;
    std::string container_frame_id;
    std::string surface_frame_id;
    double meters_per_second{ 0.0 };
  };

  struct BinData
  {
    std::string name;
    std::string frame_id;
  };

  struct TableData
  {
    std::string name;
    std::string frame_id;
  };

  struct HumanProximitySensorData
  {
    std::string human_proximity_sensor_id;
    std::string topic_table;
    std::string topic_pen;
    tijmath::RelativePose3 pose_table;
    tijmath::RelativePose3 pose_pen;
  };

  struct PickAndPlacePoseHintsData
  {
    tijmath::RelativePose3 target_pose;
    tijmath::RelativePose3 approach_pose;
  };

  virtual const std::string& getWorldFrameId() const = 0;

  virtual const tijmath::RelativePose3& getDropBucketPose() const = 0;

  virtual const tijmath::RelativePose3& getGripperToolSwappingTablePose() const = 0;

  virtual const std::vector<BinData>& getListOfBins() const = 0;

  virtual const std::vector<TableData>& getListOfTables() const = 0;

  virtual const std::vector<DeviceData>& getListOfLogicalCameras() const = 0;

  virtual const std::vector<DeviceData>& getListOfQualityControlSensors() const = 0;

  virtual const std::vector<DeviceData>& getListOfAgvs() const = 0;

  virtual const std::vector<DeviceData>& getListOfAssemblyStations() const = 0;

  virtual const std::vector<ConveyorBeltData>& getListOfConveyorBelts() const = 0;

  virtual const std::vector<tijmath::RelativePose3>& getListOfSafeWaitingSpotHints() const = 0;

  virtual const std::vector<HumanProximitySensorData>& getListOfHumanProximitySensors() const = 0;

  virtual const std::vector<PickAndPlacePoseHintsData>&
  getListOfApproachHints(const std::string& robot_name) const = 0;
};

}  // namespace tijcore
