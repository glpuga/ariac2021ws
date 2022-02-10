/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>
#include <string>

// gtest
#include "gmock/gmock.h"

// tijcore
#include <tijcore/perception/ModelContainerInterface.hpp>

namespace tijcore
{
class ModelContainerMock : public ModelContainerInterface
{
public:
  using Ptr = std::unique_ptr<ModelContainerMock>;

  ModelContainerMock(const std::string& name, const std::string& container_reference_frame_id,
                     const std::string& surface_reference_frame_id, const tijmath::RelativePose3& pose,
                     const CuboidVolume& container_volume, const std::string& shared_workspace_id)
    : ModelContainerInterface{
      name, container_reference_frame_id, surface_reference_frame_id, pose, container_volume, shared_workspace_id
    }
  {
  }

  MOCK_CONST_METHOD0(enabled, bool());

  MOCK_CONST_METHOD0(isSubmissionTray, bool());

  MOCK_METHOD1(setEnabled, void(const bool state));

  MOCK_CONST_METHOD0(region, WorkRegionId());
};

}  // namespace tijcore
