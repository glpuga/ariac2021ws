/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// gtest
#include "gmock/gmock.h"

// tijcore
#include <tijcore/perception/ModelContainerInterface.hpp>

namespace tijcore {

class ModelContainerMock : public ModelContainerInterface {
public:
  using Ptr = std::unique_ptr<ModelContainerMock>;

  ModelContainerMock(const std::string &name, const std::string &local_frame_id,
                     const RelativePose3 &pose,
                     const CuboidVolume &container_volume,
                     const std::string &shared_workspace_id)
      : ModelContainerInterface{name, local_frame_id, pose, container_volume,
                                shared_workspace_id} {}
  MOCK_CONST_METHOD0(enabled, bool());

  MOCK_CONST_METHOD0(isSubmissionTray, bool());

  MOCK_METHOD1(setEnabled, void(const bool state));

  MOCK_CONST_METHOD0(region, WorkRegionId());
};

} // namespace tijcore