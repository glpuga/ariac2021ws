/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// gtest
#include "gmock/gmock.h"

// tijcore
#include <tijcore/perception/PickAndPlaceRobotInterface.hpp>

namespace tijcore {

class PickAndPlaceRobotMock : public PickAndPlaceRobotInterface {
public:
  using Ptr = std::unique_ptr<PickAndPlaceRobotMock>;

  MOCK_CONST_METHOD0(getInSafePose, bool());

  MOCK_CONST_METHOD1(getInSafePoseNearTarget,
                     bool(const tijcore::RelativePose3 &target));

  MOCK_CONST_METHOD1(getToGraspingPoseHint,
                     bool(const tijcore::RelativePose3 &target));

  MOCK_CONST_METHOD1(getInLandingSpot,
                     bool(const tijcore::RelativePose3 &target));

  MOCK_CONST_METHOD0(graspPartFromAbove, bool());

  MOCK_CONST_METHOD0(placePartFromAbove, bool());

  MOCK_CONST_METHOD1(twistPartInPlace, bool(tijcore::RelativePose3 &target,
                                            const TwistDirection &direction));

  MOCK_CONST_METHOD0(enabled, bool());

  MOCK_CONST_METHOD0(supportedRegions, std::set<WorkRegionId>());

  MOCK_CONST_METHOD0(name, std::string());

  MOCK_CONST_METHOD0(getRobotPlanningGroup, std::string());

  MOCK_CONST_METHOD0(gripperHasPartAttached, bool());

  MOCK_CONST_METHOD1(setSuctionGripper, void(const bool state));

  MOCK_CONST_METHOD1(patchJointStateValuesForRestingPose,
                     void(std::vector<double> &));

  MOCK_CONST_METHOD2(patchJointStateValuesToGetCloseToTarget,
                     void(std::vector<double> &joint_states,
                          const tijcore::RelativePose3 &target));

  MOCK_CONST_METHOD2(patchJointStateValuesGraspingHingPoseNearTarget,
                     void(std::vector<double> &joint_states,
                          const tijcore::RelativePose3 &target));

  MOCK_CONST_METHOD1(patchJointStateValuesForAlignedZeroWrist,
                     void(std::vector<double> &));

  MOCK_METHOD0(cancelAction, void());

  MOCK_CONST_METHOD0(dropPartWhereYouStand, bool());

  MOCK_METHOD1(markAsInaccessible,
               void(const std::vector<ModelTraySharedAccessSpaceDescription>
                        &descriptors));

  MOCK_METHOD1(markAsAccessible,
               void(const std::vector<ModelTraySharedAccessSpaceDescription>
                        &descriptors));
};

} // namespace tijcore
