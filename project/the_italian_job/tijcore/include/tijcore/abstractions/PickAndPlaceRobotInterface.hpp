/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>
#include <set>
#include <string>
#include <vector>

// tijcore
#include <tijcore/datatypes/PartTypeId.hpp>
#include <tijmath/RelativePose3.hpp>

namespace tijcore
{
class PickAndPlaceRobotInterface
{
public:
  using Ptr = std::unique_ptr<PickAndPlaceRobotInterface>;
  using SharedPtr = std::shared_ptr<PickAndPlaceRobotInterface>;

  virtual ~PickAndPlaceRobotInterface() = default;

  virtual bool getArmInRestingPose() const = 0;

  virtual bool getInSafePoseNearTarget(const tijmath::RelativePose3& target) const = 0;

  virtual bool getInLandingSpot(const tijmath::RelativePose3& target) const = 0;

  virtual bool contactPartFromAboveAndGrasp(const tijmath::RelativePose3& target,
                                            const tijcore::PartTypeId& part_type_id) const = 0;

  virtual bool placePartFromAbove(const tijmath::RelativePose3& target,
                                  const tijcore::PartTypeId& part_type_id) const = 0;

  virtual bool turnOnGripper() const = 0;

  virtual bool turnOffGripper() const = 0;

  virtual bool gripperHasPartAttached() const = 0;

  virtual bool enabled() const = 0;

  virtual std::string name() const = 0;

  virtual void abortCurrentAction() const = 0;

protected:
  virtual std::string getRobotPlanningGroup() const = 0;

  virtual void setSuctionGripper(const bool state) const = 0;

  // TODO(glpuga) these three might better belong in the common implementation
  // class.
  virtual void patchJointStateValuesForArmInRestingPose(std::vector<double>&) const = 0;

  virtual void patchJointStateValuesToGetCloseToTargetPose(
      std::vector<double>& joint_states, const tijmath::RelativePose3& target) const = 0;

  virtual void patchJointStateValuesGraspingHingPoseNearTarget(
      std::vector<double>& joint_states, const tijmath::RelativePose3& target) const = 0;
};

}  // namespace tijcore
