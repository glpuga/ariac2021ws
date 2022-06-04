/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <set>
#include <string>
#include <vector>

// gtest
#include "gmock/gmock.h"

// tijcore
#include <tijcore/abstractions/ResourceManagerInterface.hpp>

namespace tijcore
{
class ResourceManagerMock : public ResourceManagerInterface
{
public:
  MOCK_METHOD1(findVacantLociCandidates, std::vector<ManagedLocusHandle>(const double free_radius));

  MOCK_METHOD1(getPartSourceListByType, std::vector<ManagedLocusHandle>(const PartId& part_id));

  MOCK_METHOD1(getMovableTraySourceListByType,
               std::vector<ManagedLocusHandle>(const MovableTrayId& movable_tray_type));

  MOCK_METHOD1(findSiblingLociByCommonParent,
               std::vector<ManagedLocusHandle>(const std::string& part_id));

  MOCK_METHOD1(returnActiveHandleCountForParentContainer,
               std::size_t(const std::string& parent_name));

  MOCK_METHOD1(
      getPickAndPlaceRobotHandle,
      std::optional<PickAndPlaceRobotHandle>(const std::vector<tijmath::RelativePose3>& waypoints));

  MOCK_METHOD1(getLocusAtPose,
               std::optional<ManagedLocusHandle>(const tijmath::RelativePose3& pose));

  MOCK_METHOD1(processInputSensorData, void(const std::vector<ObservedItem>& observed_models));
};

}  // namespace tijcore
