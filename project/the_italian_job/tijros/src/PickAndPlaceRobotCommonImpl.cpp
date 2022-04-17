/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

// ros
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>

// project
#include <tijlogger/logger.hpp>
#include <tijmath/math_utilities.hpp>
#include <tijros/PickAndPlaceRobotCommonImpl.hpp>
#include <tijros/utils/utils.hpp>

namespace tijros
{
namespace
{
// TODO(glpuga) this frame id is hardcoded in multiple places. Use the one that
// can be get through the scene configuration in  the toolbox
static char world_frame[] = "world";

static const double landing_pose_height_ = 0.30;
static const double pick_search_length_ = 0.06;
static const double part_drop_height_ = 0.04;

static const double pickup_displacement_jump_threshold_ = 10.0;
static const double pickup_displacement_step_ = 0.0025;
static const double max_planning_time_ = 20.0;
static const int max_planning_attempts_ = 5;

static const double tight_goal_position_tolerance_ = 0.001;
static const double tight_goal_orientation_tolerance_ =
    tijmath::utils::angles::degreesToRadians(0.2);

static const double coarse_goal_position_tolerance_ = 0.015;
static const double coarse_goal_orientation_tolerance_ =
    tijmath::utils::angles::degreesToRadians(2.5);

moveit_msgs::CollisionObject
createCollisionBox(const std::string& id, const std::string& sub_id, const std::string& frame_id,
                   const double width, const double height, const double depth,
                   const double xoffset, const double yoffset, const double zoffset,
                   const int operation = moveit_msgs::CollisionObject::ADD)
{
  // do the hard part
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = id + "_" + sub_id;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = width;
  primitive.dimensions[1] = height;
  primitive.dimensions[2] = depth;

  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = xoffset;
  box_pose.position.y = yoffset;
  box_pose.position.z = zoffset;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = operation;

  return collision_object;
}

double estimatePartHeight(const tijmath::Matrix3& orientation_in_world,
                          const tijcore::PartTypeId& id)
{
  const auto part_dimensions = tijcore::part_type::dimensions(id);
  const double estimated_height =
      std::abs(tijmath::Vector3{ 0, 0, 1 }.dot(part_dimensions[0] * orientation_in_world.col(0) +
                                               part_dimensions[1] * orientation_in_world.col(1) +
                                               part_dimensions[2] * orientation_in_world.col(2)));
  return estimated_height;
}

}  // namespace

PickAndPlaceRobotCommonImpl::PickAndPlaceRobotCommonImpl(const tijcore::Toolbox::SharedPtr& toolbox)
  : toolbox_{ toolbox }
{
}

void PickAndPlaceRobotCommonImpl::useNarrowTolerances(const bool tight_mode) const
{
  if (tight_mode)
  {
    move_group_ptr_->setGoalPositionTolerance(tight_goal_position_tolerance_);
    move_group_ptr_->setGoalOrientationTolerance(tight_goal_orientation_tolerance_);
  }
  else
  {
    move_group_ptr_->setGoalPositionTolerance(coarse_goal_position_tolerance_);
    move_group_ptr_->setGoalOrientationTolerance(coarse_goal_orientation_tolerance_);
  }
}

moveit::planning_interface::MoveGroupInterface*
PickAndPlaceRobotCommonImpl::buildMoveItGroupHandle() const
{
  // if the setup had not been performed before, do it now
  // Ugly AF, need to find a better solution.

  // TODO(glpuga) for some reason, keeping state in this object causes
  // errors in one of the joints, in both robots(!)
  const auto custom_moveit_namespace = "/ariac/custom/" + name();

  move_group_ptr_.reset();
  if (!move_group_ptr_)
  {
    moveit::planning_interface::MoveGroupInterface::Options options{
      getRobotPlanningGroup(), custom_moveit_namespace + "/robot_description",
      ros::NodeHandle(custom_moveit_namespace)
    };
    move_group_ptr_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(options);
    move_group_ptr_->setPlanningTime(max_planning_time_);
    move_group_ptr_->setNumPlanningAttempts(max_planning_attempts_);

    // default to coarse tolerances
    useNarrowTolerances(false);

    // moveit seems to be defaulting to a value less than one now...
    move_group_ptr_->setMaxAccelerationScalingFactor(1.0);
    move_group_ptr_->setMaxVelocityScalingFactor(1.0);
  }

  if (!planning_scene_ptr_)
  {
    planning_scene_ptr_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>(
        custom_moveit_namespace);
    buildObstacleSceneFromDescription();
  }

  return move_group_ptr_.get();
}

bool PickAndPlaceRobotCommonImpl::getArmInRestingPose() const
{
  const auto action_name = "getArmInRestingPose";
  auto move_group_ptr = buildMoveItGroupHandle();
  const robot_state::JointModelGroup* joint_model_group =
      move_group_ptr->getCurrentState()->getJointModelGroup(getRobotPlanningGroup());
  {
    INFO("{}: {} is calculating the target", action_name, name());
    moveit::core::RobotStatePtr current_state = move_group_ptr->getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    patchJointStateValuesForArmInRestingPose(joint_group_positions);
    move_group_ptr->setJointValueTarget(joint_group_positions);
    move_group_ptr->setStartState(*move_group_ptr->getCurrentState());
  }
  moveit::planning_interface::MoveGroupInterface::Plan movement_plan;
  {
    INFO("{}: {} is planning", action_name, name());
    auto success = (move_group_ptr->plan(movement_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
    {
      ERROR("{}: {} failed to generate a plan", action_name, name());
      return false;
    }
  }
  {
    INFO("{}: {} is executing", action_name, name());
    auto success = (move_group_ptr->execute(movement_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
    {
      ERROR("{}: {} failed to execute", action_name, name());
      return false;
    }
  }
  INFO("{}: {} finished execution", action_name, name());
  return true;
}

bool PickAndPlaceRobotCommonImpl::getInSafePoseNearTarget(
    const tijmath::RelativePose3& target) const
{
  const auto action_name = "getInSafePoseNearTarget";
  if (!enabled())
  {
    INFO("{}: {} failed to execute because the robot is disabled", action_name, name());
    return false;
  }
  auto move_group_ptr = buildMoveItGroupHandle();
  const robot_state::JointModelGroup* joint_model_group =
      move_group_ptr->getCurrentState()->getJointModelGroup(getRobotPlanningGroup());
  {
    INFO("{}: {} is calculating the target", action_name, name());
    moveit::core::RobotStatePtr current_state = move_group_ptr->getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group,

                                           joint_group_positions);
    patchJointStateValuesForArmInRestingPose(joint_group_positions);
    patchJointStateValuesToGetCloseToTargetPose(joint_group_positions, target);

    move_group_ptr->setJointValueTarget(joint_group_positions);
    move_group_ptr->setStartState(*move_group_ptr->getCurrentState());
  }
  moveit::planning_interface::MoveGroupInterface::Plan movement_plan;
  {
    INFO("{}: {} is planning", action_name, name());
    auto success = (move_group_ptr->plan(movement_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
    {
      ERROR("{}: {} failed to generate a plan", action_name, name());
      return false;
    }
  }
  {
    INFO("{}: {} is executing", action_name, name());
    auto success = (move_group_ptr->execute(movement_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
    {
      ERROR("{}: {} failed to execute", action_name, name());
      return false;
    }
  }
  INFO("{}: {} finished execution", action_name, name());
  return true;
}

bool PickAndPlaceRobotCommonImpl::contactPartFromAboveAndGrasp(
    const tijmath::RelativePose3& target, const tijcore::PartTypeId& part_type_id) const
{
  const auto action_name = "contactPartFromAboveAndGrasp";
  if (!enabled())
  {
    INFO("{}: {} failed to execute because the robot is disabled", action_name, name());
    return false;
  }
  auto move_group_ptr = buildMoveItGroupHandle();

  INFO("{}: {} is calculating the approximation trajectory", action_name, name());
  // configure tighter tolerances for this
  useNarrowTolerances(true);
  // make sure we trade in world poses only
  move_group_ptr->setPoseReferenceFrame(world_frame);
  // turn on the gripper
  setSuctionGripper(true);

  auto frame_transformer = toolbox_->getFrameTransformer();
  const auto target_in_world_pose = frame_transformer->transformPoseToFrame(target, world_frame);

  // TODO(glpuga) this should better be an function that given the part pose,
  // returns the estimated gripper pose
  auto end_effector_target_pose = target_in_world_pose;
  alignEndEffectorWithTarget(end_effector_target_pose);

  auto end_effector_target_pose_in_world = end_effector_target_pose.pose();

  // notice that part poses get detected with a height equal to about half the
  // height of the piece
  const auto run_top =
      end_effector_target_pose_in_world.position().vector().z() +
      estimatePartHeight(target_in_world_pose.rotation().rotationMatrix(), part_type_id) * 0.5 +
      pick_search_length_ * 0.5;
  const auto run_bottom =
      end_effector_target_pose_in_world.position().vector().z() +
      estimatePartHeight(target_in_world_pose.rotation().rotationMatrix(), part_type_id) * 0.5 -
      pick_search_length_ * 0.5;
  end_effector_target_pose_in_world.position().vector().z() = run_top;

  while (!gripperHasPartAttached() &&
         (end_effector_target_pose_in_world.position().vector().z() > run_bottom))
  {
    // decrease the gripper one step
    end_effector_target_pose_in_world.position().vector().z() -= pickup_displacement_step_;

    INFO("{}: {}  is generating the step moveit plan", action_name, name());
    moveit::planning_interface::MoveGroupInterface::Plan movement_plan;
    {
      auto target_geo_pose_in_world =
          utils::convertCorePoseToGeoPose(end_effector_target_pose_in_world);
      move_group_ptr->setPoseTarget(target_geo_pose_in_world);
      move_group_ptr->setStartState(*move_group_ptr->getCurrentState());

      bool success = (move_group_ptr->plan(movement_plan) ==
                      moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (!success)
      {
        ERROR("{}: {} failed to generate a plan for step, aborting grasping", action_name, name());
        return false;
      }
    }

    INFO("{}: {} is executing the step moveit plan", action_name, name());
    move_group_ptr->execute(movement_plan);
  }

  if (gripperHasPartAttached())
  {
    INFO("{}: {} succeeded to grasp the part", action_name, name());
    return true;
  }

  INFO("{}: {} failed to grasp the part", action_name, name());
  return true;
}

bool PickAndPlaceRobotCommonImpl::placePartFromAbove(const tijmath::RelativePose3& target,
                                                     const tijcore::PartTypeId& part_type_id) const
{
  const auto action_name = "placePartFromAbove";
  if (!enabled())
  {
    INFO("{}: {} failed to execute because the robot is disabled", action_name, name());
    return false;
  }
  auto move_group_ptr = buildMoveItGroupHandle();

  INFO("Place movement: {} is calculating the pickup trajectory", name());

  // make sure we trade in world poses only
  move_group_ptr->setPoseReferenceFrame(world_frame);

  auto frame_transformer = toolbox_->getFrameTransformer();

  const auto target_in_world_pose = frame_transformer->transformPoseToFrame(target, world_frame);

  // TODO(glpuga) this should better be a function that given the part pose,
  // returns the estimated grasping pose
  auto end_effector_target_pose = target_in_world_pose;
  alignEndEffectorWithTarget(end_effector_target_pose);

  auto end_effector_target_pose_in_world = end_effector_target_pose.pose();

  end_effector_target_pose_in_world.position().vector().z() +=
      part_drop_height_ +
      estimatePartHeight(target_in_world_pose.rotation().rotationMatrix(), part_type_id);

  {
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(utils::convertCorePoseToGeoPose(end_effector_target_pose_in_world));

    INFO("Place will drop the part once we are at {} in {} frame",
         utils::convertGeoPoseToCorePose(waypoints.at(0)), world_frame);

    INFO("Place movement: {} is generating the moveit plan", name());
    moveit_msgs::RobotTrajectory trajectory;
    {
      const double fraction = move_group_ptr->computeCartesianPath(
          waypoints, pickup_displacement_step_, pickup_displacement_jump_threshold_, trajectory);
      DEBUG("Place movement: cartesian planner plan fraction for {}: {}", name(), fraction);
      bool success = (fraction > 0.95);
      if (!success)
      {
        ERROR(
            "{} failed to generate a plan to the drop point "
            "(fraction: {})",
            name(), fraction);
        WARNING("{} will release the part here");
        setSuctionGripper(false);
        return false;
      }
    }

    // we don't check the return value because it's going to return failure once
    // we hit the table
    INFO("Place movement: {} is executing the movement", name());
    move_group_ptr->execute(trajectory);
  }

  setSuctionGripper(false);
  INFO("{} placed the part that the destination", name());
  return true;
}

bool PickAndPlaceRobotCommonImpl::turnOnGripper() const
{
  const auto action_name = "turnOnGripper";
  INFO("{}: {} is turning on the gripper", action_name, name());
  setSuctionGripper(true);
  return true;
}

bool PickAndPlaceRobotCommonImpl::turnOffGripper() const
{
  const auto action_name = "turnOffGripper";
  INFO("{}: {} is turning off the gripper", action_name, name());
  setSuctionGripper(false);
  return true;
}

void PickAndPlaceRobotCommonImpl::abortCurrentAction() const
{
  const auto action_name = "abortCurrentAction";
  auto move_group_ptr = buildMoveItGroupHandle();
  WARNING("{}: {} is aborting", action_name, name());
  move_group_ptr->stop();
}

void PickAndPlaceRobotCommonImpl::buildObstacleSceneFromDescription() const
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  const int operation = moveit_msgs::CollisionObject::ADD;
  auto scene_configuration = toolbox_->getSceneConfigReader();
  const double z_offset{ 0.025 };

  DEBUG(" Generating collision scene");
  DEBUG(" - adding AGV tray representatives");
  for (const auto& item : scene_configuration->getListOfAgvs())
  {
    collision_objects.push_back(createCollisionBox(item.name, "surface", item.frame_id, 0.5, 0.7,
                                                   z_offset, 0, 0, 0, operation));
    collision_objects.push_back(createCollisionBox(item.name, "tower_foot", item.frame_id, 0.23,
                                                   0.23, 0.22, 0.0, -0.45, 0.11, operation));
    collision_objects.push_back(createCollisionBox(item.name, "tower_head", item.frame_id, 0.15,
                                                   0.15, 0.7, 0.0, -0.45, 0.35, operation));
  }

  DEBUG(" - adding assembly station representatives");
  for (const auto& item : scene_configuration->getListOfAssemblyStations())
  {
    collision_objects.push_back(createCollisionBox(item.name, "briefcase_table", item.frame_id, 1.6,
                                                   1.2, z_offset, -0.3, -0.1, 0.0, operation));
    collision_objects.push_back(createCollisionBox(item.name, "briefcase_front", item.frame_id,
                                                   0.05, 0.6, 0.16, 0.3, 0.0, 0.08, operation));
    collision_objects.push_back(createCollisionBox(item.name, "briefcase_side", item.frame_id, 0.6,
                                                   0.05, 0.16, 0.0, -0.3, 0.08, operation));
    collision_objects.push_back(createCollisionBox(item.name, "table_right", item.frame_id, 1.0,
                                                   0.10, 1.0, -0.70, -0.75, 0.5, operation));
    collision_objects.push_back(createCollisionBox(item.name, "table_back", item.frame_id, 0.8, 1.2,
                                                   1.0, -0.7, -0.1, 0.5, operation));
    collision_objects.push_back(createCollisionBox(item.name, "table_left", item.frame_id, 1.6, 0.1,
                                                   1.0, -0.45, 0.54, 0.5, operation));
    collision_objects.push_back(createCollisionBox(item.name, "briefcase_top_1", item.frame_id,
                                                   0.04, 0.1, 0.80, 0.33, 0.35, 0.40, operation));
    collision_objects.push_back(createCollisionBox(item.name, "briefcase_top_2", item.frame_id, 0.7,
                                                   0.1, 0.1, 0.0, 0.35, 0.75, operation));
  }

  DEBUG(" - adding bin representatives");
  for (const auto& item : scene_configuration->getListOfBins())
  {
    collision_objects.push_back(createCollisionBox(item.name, "surface", item.frame_id, 0.6, 0.6,
                                                   z_offset, 0, 0, 0, operation));

    collision_objects.push_back(createCollisionBox(item.name, "wall_1", item.frame_id, 0.64, 0.06,
                                                   0.08, 0.0, 0.31, 0.04, operation));
    collision_objects.push_back(createCollisionBox(item.name, "wall_2", item.frame_id, 0.64, 0.06,
                                                   0.08, 0.0, -0.31, 0.04, operation));
    collision_objects.push_back(createCollisionBox(item.name, "wall_3", item.frame_id, 0.06, 0.64,
                                                   0.08, 0.31, 0.0, 0.04, operation));
    collision_objects.push_back(createCollisionBox(item.name, "wall_4", item.frame_id, 0.06, 0.64,
                                                   0.08, -0.31, 0.0, 0.04, operation));
  }

  DEBUG(" - adding conveyor belt representatives");
  for (const auto& item : scene_configuration->getListOfConveyorBelts())
  {
    collision_objects.push_back(createCollisionBox(item.name, "surface", item.container_frame_id,
                                                   0.63, 9, z_offset, 0, 0, 0, operation));
  }

  // kitting rail
  collision_objects.push_back(
      createCollisionBox("kitting", "rail", "world", 0.4, 10.0, 0.10, -1.3, 0.0, 0.93, operation));

  // Imaginary divider
  collision_objects.push_back(
      createCollisionBox("divider", "rail", "world", 0.05, 10.0, 0.1, -1.4, 0.0, 2.7, operation));

  planning_scene_ptr_->applyCollisionObjects(collision_objects);
}

void PickAndPlaceRobotCommonImpl::alignEndEffectorWithTarget(
    tijmath::RelativePose3& end_effector_target_pose) const
{
  const auto orientation = end_effector_target_pose.rotation().rotationMatrix();
  const auto x_director = tijmath::Vector3{ 0, 0, -1 };

  auto original_x_director = orientation.col(0);
  auto original_y_director = orientation.col(1);
  auto original_z_director = orientation.col(2);

  // to try to consistently align with the same axis, which is important for
  // part flipping, try to use always the same vector, unless that's the one
  // that's pointing up.
  tijmath::Vector3 z_director;
  if ((std::abs(x_director.dot(original_x_director)) >
       std::abs(x_director.dot(original_y_director))) &&
      (std::abs(x_director.dot(original_x_director)) >
       std::abs(x_director.dot(original_z_director))))
  {
    // x is pointing up
    // TODO(glpuga) the sign inversion is because without this accessing the
    // briefcases to remove a battery fails because the full body of the robot
    // is oriented to try to match the orientation of the piece, resulting in
    // unfeasable plans. It's likely related to the end of range of the wrist
    // articulations.
    z_director = orientation.col(2) * (-1);
  }
  else
  {
    // is horizontal, always choose x
    // TODO(glpuga) the sign inversion is because without this accessing the
    // briefcases to remove a battery fails because the full body of the robot
    // is oriented to try to match the orientation of the piece, resulting in
    // unfeasable plans. It's likely related to the end of range of the wrist
    // articulations.
    z_director = orientation.col(0) * (-1);
  }

  // Don't assume the target will be perfectly normal to the world Z axis
  z_director -= x_director * z_director.dot(x_director);
  z_director = z_director / z_director.norm();

  auto y_director = z_director.cross(x_director);
  y_director = y_director / y_director.norm();

  const auto end_effector_orientation =
      tijmath::Matrix3{
        x_director,
        y_director,
        z_director,
      }
          .trans();

  end_effector_target_pose.rotation() = tijmath::Rotation{ end_effector_orientation };
}

}  // namespace tijros
