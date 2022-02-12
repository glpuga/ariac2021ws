/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>
#include <set>
#include <string>
#include <vector>

// tijcore
#include <tijcore/agents/PartTypeId.hpp>
#include <tijcore/agents/WorkRegionId.hpp>
#include <tijcore/perception/ModelTraySharedAccessSpaceDescription.hpp>
#include <tijmath/RelativePose3.hpp>

namespace tijcore
{
class PickAndPlaceRobotInterface
{
public:
  using Ptr = std::unique_ptr<PickAndPlaceRobotInterface>;
  using SharedPtr = std::shared_ptr<PickAndPlaceRobotInterface>;

  virtual ~PickAndPlaceRobotInterface() = default;

  virtual bool getInSafePose() const = 0;

  virtual bool getInSafePoseNearTarget(const tijmath::RelativePose3& target) const = 0;

  virtual bool getToGraspingPoseHint(const tijmath::RelativePose3& target) const = 0;

  virtual bool getInLandingSpot(const tijmath::RelativePose3& target) const = 0;

  virtual bool graspPartFromAbove(const tijmath::RelativePose3& target,
                                  const tijcore::PartTypeId& part_type_id) const = 0;

  virtual bool placePartFromAbove(const tijmath::RelativePose3& target,
                                  const tijcore::PartTypeId& part_type_id) const = 0;

  virtual bool twistPartInPlace(tijmath::RelativePose3& target, const tijcore::PartTypeId& part_type_id) const = 0;

  virtual bool dropPartWhereYouStand() const = 0;

  virtual bool gripperHasPartAttached() const = 0;

  virtual bool enabled() const = 0;

  virtual std::set<WorkRegionId> supportedRegions() const = 0;

  virtual std::string name() const = 0;

  virtual void cancelAction() = 0;

  virtual void markAsInaccessible(const std::vector<ModelTraySharedAccessSpaceDescription>& descriptors) = 0;

  virtual void markAsAccessible(const std::vector<ModelTraySharedAccessSpaceDescription>& descriptors) = 0;

protected:
  virtual std::string getRobotPlanningGroup() const = 0;

  virtual void setSuctionGripper(const bool state) const = 0;

  // TODO(glpuga) these three might better belong in the common implementation
  // class.
  virtual void patchJointStateValuesForRestingPose(std::vector<double>&) const = 0;

  virtual void patchJointStateValuesToGetCloseToTarget(std::vector<double>& joint_states,
                                                       const tijmath::RelativePose3& target) const = 0;

  virtual void patchJointStateValuesGraspingHingPoseNearTarget(std::vector<double>& joint_states,
                                                               const tijmath::RelativePose3& target) const = 0;

  virtual void patchJointStateValuesForAlignedZeroWrist(std::vector<double>& joint_states) const = 0;
};

}  // namespace tijcore
