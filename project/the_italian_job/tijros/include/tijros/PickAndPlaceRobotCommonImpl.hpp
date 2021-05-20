/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <mutex>

// roscpp
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>

// tijcore
#include <tijcore/localization/RelativePose3.hpp>
#include <tijcore/perception/ModelTraySharedAccessSpaceDescription.hpp>
#include <tijcore/perception/PickAndPlaceRobotInterface.hpp>
#include <tijcore/perception/Toolbox.hpp>

namespace tijros {

class PickAndPlaceRobotCommonImpl : public tijcore::PickAndPlaceRobotInterface {
public:
  PickAndPlaceRobotCommonImpl(const tijcore::Toolbox::SharedPtr &toolbox);

  bool getInSafePose() const override;

  bool
  getInSafePoseNearTarget(const tijcore::RelativePose3 &target) const override;

  bool
  getToGraspingPoseHint(const tijcore::RelativePose3 &target) const override;

  bool getInLandingSpot(const tijcore::RelativePose3 &target) const override;

  bool graspPartFromAbove(const tijcore::RelativePose3 &target) const override;

  bool placePartFromAbove(const tijcore::RelativePose3 &target) const override;

  bool twistPartInPlace(tijcore::RelativePose3 &target,
                        const TwistDirection &direction) const override;

  bool dropPartWhereYouStand() const override;

  void cancelAction() override;

  void markAsInaccessible(
      const std::vector<tijcore::ModelTraySharedAccessSpaceDescription>
          &descriptors) override;

  void markAsAccessible(
      const std::vector<tijcore::ModelTraySharedAccessSpaceDescription>
          &descriptors) override;

private:
  tijcore::Toolbox::SharedPtr toolbox_;

  mutable std::unique_ptr<moveit::planning_interface::MoveGroupInterface>
      move_group_ptr_;
  mutable std::unique_ptr<moveit::planning_interface::PlanningSceneInterface>
      planning_scene_ptr_;

  moveit::planning_interface::MoveGroupInterface *
  getMoveItGroupHandlePtr() const;

  void setupObjectConstraints() const;

  void markAsCommanded(
      const std::vector<tijcore::ModelTraySharedAccessSpaceDescription>
          &descriptors,
      const int command);

  void
  alignEndEffectorWithTarget(tijcore::RelativePose3 &target_in_world) const;
};

} // namespace tijros
