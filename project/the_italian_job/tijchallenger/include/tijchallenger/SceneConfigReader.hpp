/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <string>
#include <vector>

// project
#include <tijcore/perception/SceneConfigReaderInterface.hpp>

namespace tijchallenger
{
class SceneConfigReader : public tijcore::SceneConfigReaderInterface
{
public:
  const std::string& getWorldFrameId() const override;

  const tijcore::RelativePose3& getDropBucketPose() const override;

  const std::vector<BinData>& getListOfBins() const override;

  const std::vector<DeviceData>& getListOfLogicalCameras() const override;

  const std::vector<DeviceData>& getListOfQualityControlSensors() const override;

  const std::vector<DeviceData>& getListOfAgvs() const override;

  const std::vector<DeviceData>& getListOfAssemblyStations() const override;

  const std::vector<ConveyorBeltData>& getListOfConveyorBelts() const override;

  const std::vector<tijcore::ModelTraySharedAccessSpaceDescription>&
  getListOfSharedAccessSpaceDescriptions() const override;

  const std::vector<tijcore::RelativePose3>& getListOfGantryPlanningHints() const override;

  const std::vector<tijcore::RelativePose3>& getListOfSafeWaitingSpotHints() const override;
};

};  // namespace tijchallenger
