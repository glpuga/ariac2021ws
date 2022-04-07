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

  MOCK_METHOD1(updateSensorData, void(const std::vector<ObservedModel>& observed_models));

  MOCK_METHOD3(getModelTrayExclusionZoneHandle,
               std::optional<ExclusionZoneHandle>(
                   const std::string& model_container_name,
                   const std::optional<std::string>& additional_model_tray_name_opt,
                   const std::chrono::milliseconds& timeout));

  MOCK_METHOD1(getSubmissionTray,
               std::optional<SubmissionTrayHandle>(const std::string& model_container_name));

  MOCK_CONST_METHOD1(getContainerExclusionZoneId,
                     std::string(const std::string& model_container_name));

  MOCK_CONST_METHOD1(getWorkRegionId, WorkRegionId(const ManagedLocusHandle& handle));

  MOCK_CONST_METHOD0(getListOfExclusionZones,
                     const std::vector<ModelTraySharedAccessSpaceDescription>&());
};

}  // namespace tijcore
