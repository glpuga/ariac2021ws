/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <string>
#include <vector>

// project
#include <tijcore/abstractions/SceneConfigReaderInterface.hpp>

namespace tijchallenger
{
class SceneConfigReader : public tijcore::SceneConfigReaderInterface
{
public:
  const std::string& getWorldFrameId() const override;

  const tijmath::RelativePose3& getDropBucketPose() const override;

  const tijmath::RelativePose3& getGripperToolSwappingTablePose() const override;

  const std::vector<BinData>& getListOfBins() const override;

  const std::vector<TableData>& getListOfTables() const override;

  const std::vector<DeviceData>& getListOfLogicalCameras() const override;

  const std::vector<DeviceData>& getListOfQualityControlSensors() const override;

  const std::vector<DeviceData>& getListOfAgvs() const override;

  const std::vector<DeviceData>& getListOfAssemblyStations() const override;

  const std::vector<ConveyorBeltData>& getListOfConveyorBelts() const override;

  const std::vector<tijmath::RelativePose3>& getListOfSafeWaitingSpotHints() const override;

  const std::vector<HumanProximitySensorData>& getListOfHumanProximitySensors() const override;

  const std::vector<PickAndPlacePoseHintsData>&
  getListOfApproachHints(const std::string& robot_name) const override;
};

};  // namespace tijchallenger
