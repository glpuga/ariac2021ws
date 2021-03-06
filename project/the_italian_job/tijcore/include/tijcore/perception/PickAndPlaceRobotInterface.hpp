/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>
#include <set>

// tijcore
#include <tijcore/agents/WorkRegionId.hpp>
#include <tijcore/localization/RelativePose3.hpp>
#include <tijcore/perception/ModelTraySharedAccessSpaceDescription.hpp>

namespace tijcore {

class PickAndPlaceRobotInterface {
public:
  using Ptr = std::unique_ptr<PickAndPlaceRobotInterface>;
  using SharedPtr = std::shared_ptr<PickAndPlaceRobotInterface>;

  virtual ~PickAndPlaceRobotInterface() = default;

  virtual bool getInSafePose() const = 0;

  virtual bool
  getInSafePoseNearTarget(const tijcore::RelativePose3 &target) const = 0;

  virtual bool
  getToGraspingPoseHint(const tijcore::RelativePose3 &target) const = 0;

  virtual bool getInLandingSpot(const tijcore::RelativePose3 &target) const = 0;

  virtual bool
  graspPartFromAbove(const tijcore::RelativePose3 &target) const = 0;

  virtual bool
  placePartFromAbove(const tijcore::RelativePose3 &target) const = 0;

  virtual bool dropPartWhereYouStand() const = 0;

  virtual bool gripperHasPartAttached() const = 0;

  virtual bool enabled() const = 0;

  virtual std::set<WorkRegionId> supportedRegions() const = 0;

  virtual std::string name() const = 0;

  virtual void cancelAction() = 0;

  virtual void
  markAsInaccessible(const std::vector<ModelTraySharedAccessSpaceDescription>
                         &descriptors) = 0;

  virtual void
  markAsAccessible(const std::vector<ModelTraySharedAccessSpaceDescription>
                       &descriptors) = 0;

protected:
  virtual std::string getRobotPlanningGroup() const = 0;

  virtual void setSuctionGripper(const bool state) const = 0;

  // TODO(glpuga) these three might better belong in the common implementation
  // class.
  virtual void
  patchJointStateValuesForRestingPose(std::vector<double> &) const = 0;

  virtual void patchJointStateValuesToGetCloseToTarget(
      std::vector<double> &joint_states,
      const tijcore::RelativePose3 &target) const = 0;

  virtual void patchJointStateValuesGraspingHingPoseNearTarget(
      std::vector<double> &joint_states,
      const tijcore::RelativePose3 &target) const = 0;
};

} // namespace tijcore
