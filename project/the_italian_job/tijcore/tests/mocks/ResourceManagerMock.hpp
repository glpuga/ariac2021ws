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
  MOCK_METHOD1(findEmptyLoci, std::vector<ManagedLocusHandle>(const double free_radius));

  MOCK_METHOD1(findManagedLociByPartId, std::vector<ManagedLocusHandle>(const PartId& part_id));

  MOCK_METHOD1(findManagedLociByParent,
               std::vector<ManagedLocusHandle>(const std::string& part_id));

  MOCK_METHOD1(getPickAndPlaceRobotHandle,
               std::optional<PickAndPlaceRobotHandle>(const std::set<WorkRegionId>& regions));

  MOCK_METHOD1(getManagedLocusHandleForPose,
               std::optional<ManagedLocusHandle>(const tijmath::RelativePose3& pose));

  MOCK_METHOD1(updateSensorData, void(const std::vector<ObservedItem>& observed_models));
};

}  // namespace tijcore
