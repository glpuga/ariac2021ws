/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>
#include <mutex>
#include <vector>

// roscpp
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>

// tijcore
#include <tijcore/abstractions/PickAndPlaceRobotInterface.hpp>
#include <tijcore/coremodels/Toolbox.hpp>
#include <tijcore/resources/ModelTraySharedAccessSpaceDescription.hpp>
#include <tijmath/RelativePose3.hpp>

namespace tijros
{
class PickAndPlaceRobotCommonImpl : public tijcore::PickAndPlaceRobotInterface
{
public:
  explicit PickAndPlaceRobotCommonImpl(const tijcore::Toolbox::SharedPtr& toolbox);

  bool getInSafePose() const override;

  bool getInSafePoseNearTarget(const tijmath::RelativePose3& target) const override;

  bool getToGraspingPoseHint(const tijmath::RelativePose3& target) const override;

  bool getInLandingSpot(const tijmath::RelativePose3& target) const override;

  bool graspPartFromAbove(const tijmath::RelativePose3& target,
                          const tijcore::PartTypeId& part_type_id) const override;

  bool placePartFromAbove(const tijmath::RelativePose3& target,
                          const tijcore::PartTypeId& part_type_id) const override;

  bool twistPartInPlace(tijmath::RelativePose3& target,

                        const tijcore::PartTypeId& part_type_id) const override;

  bool dropPartWhereYouStand() const override;

  void cancelAction() override;

  void markAsInaccessible(
      const std::vector<tijcore::ModelTraySharedAccessSpaceDescription>& descriptors) override;

  void markAsAccessible(
      const std::vector<tijcore::ModelTraySharedAccessSpaceDescription>& descriptors) override;

private:
  tijcore::Toolbox::SharedPtr toolbox_;

  mutable std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_ptr_;
  mutable std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_ptr_;

  moveit::planning_interface::MoveGroupInterface* getMoveItGroupHandlePtr() const;

  void setupObjectConstraints() const;

  void
  markAsCommanded(const std::vector<tijcore::ModelTraySharedAccessSpaceDescription>& descriptors,
                  const int command);

  void alignEndEffectorWithTarget(tijmath::RelativePose3& target_in_world) const;

  void configureGoalTolerances(const bool tight_mode) const;
};

}  // namespace tijros
