/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>
#include <set>
#include <string>
#include <vector>

// gtest
#include "gmock/gmock.h"

// tijcore
#include <tijcore/abstractions/PickAndPlaceRobotInterface.hpp>

namespace tijcore
{
class PickAndPlaceRobotMock : public PickAndPlaceRobotInterface
{
public:
  using Ptr = std::unique_ptr<PickAndPlaceRobotMock>;

  MOCK_CONST_METHOD0(getInSafePose, bool());

  MOCK_CONST_METHOD1(getInSafePoseNearTarget, bool(const tijmath::RelativePose3& target));

  MOCK_CONST_METHOD1(getToGraspingPoseHint, bool(const tijmath::RelativePose3& target));

  MOCK_CONST_METHOD1(getInLandingSpot, bool(const tijmath::RelativePose3& target));

  MOCK_CONST_METHOD2(graspPartFromAbove, bool(const tijmath::RelativePose3& target,
                                              const tijcore::PartTypeId& part_type_id));

  MOCK_CONST_METHOD2(placePartFromAbove, bool(const tijmath::RelativePose3& target,
                                              const tijcore::PartTypeId& part_type_id));

  MOCK_CONST_METHOD2(twistPartInPlace,
                     bool(tijmath::RelativePose3& target, const tijcore::PartTypeId& part_type_id));

  MOCK_CONST_METHOD0(enabled, bool());

  MOCK_CONST_METHOD0(supportedRegions, std::set<WorkRegionId>());

  MOCK_CONST_METHOD0(name, std::string());

  MOCK_CONST_METHOD0(getRobotPlanningGroup, std::string());

  MOCK_CONST_METHOD0(gripperHasPartAttached, bool());

  MOCK_CONST_METHOD1(setSuctionGripper, void(const bool state));

  MOCK_CONST_METHOD1(patchJointStateValuesForRestingPose, void(std::vector<double>&));

  MOCK_CONST_METHOD2(patchJointStateValuesToGetCloseToTarget,
                     void(std::vector<double>& joint_states, const tijmath::RelativePose3& target));

  MOCK_CONST_METHOD2(patchJointStateValuesGraspingHingPoseNearTarget,
                     void(std::vector<double>& joint_states, const tijmath::RelativePose3& target));

  MOCK_CONST_METHOD1(patchJointStateValuesForAlignedZeroWrist, void(std::vector<double>&));

  MOCK_METHOD0(cancelAction, void());

  MOCK_CONST_METHOD0(dropPartWhereYouStand, bool());

  MOCK_METHOD1(markAsInaccessible,
               void(const std::vector<ModelTraySharedAccessSpaceDescription>& descriptors));

  MOCK_METHOD1(markAsAccessible,
               void(const std::vector<ModelTraySharedAccessSpaceDescription>& descriptors));
};

}  // namespace tijcore
