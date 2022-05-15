/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
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
#include <tijcore/utils/PayloadEnvelop.hpp>
#include <tijlogger/logger.hpp>
#include <tijmath/math_utilities.hpp>
#include <tijros/PickAndPlaceRobotMovements.hpp>
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

}  // namespace

PickAndPlaceRobotMovements::PickAndPlaceRobotMovements(
    tijcore::PickAndPlaceRobotSpecificInterface::Ptr robot_specific_interface,
    const tijcore::Toolbox::SharedPtr& toolbox)
  : robot_specific_interface_{ std::move(robot_specific_interface) }, toolbox_{ toolbox }
{
}

void PickAndPlaceRobotMovements::useNarrowTolerances(const bool tight_mode) const
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
PickAndPlaceRobotMovements::buildMoveItGroupHandle() const
{
  // if the setup had not been performed before, do it now
  // Ugly AF, need to find a better solution.

  // TODO(glpuga) for some reason, keeping state in this object causes
  // errors in one of the joints, in both robots(!)
  const auto custom_moveit_namespace = "/ariac/custom/" + getRobotName();

  WARNING("Creating moveit group handle for robot: {}", getRobotName());

  move_group_ptr_.reset();
  if (!move_group_ptr_)
  {
    moveit::planning_interface::MoveGroupInterface::Options options{
      robot_specific_interface_->getRobotPlanningGroup(),
      custom_moveit_namespace + "/robot_description", ros::NodeHandle(custom_moveit_namespace)
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

  planning_scene_ptr_.reset();
  if (!planning_scene_ptr_)
  {
    planning_scene_ptr_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>(
        custom_moveit_namespace);
    buildObstacleSceneFromDescription();
  }

  return move_group_ptr_.get();
}

bool PickAndPlaceRobotMovements::getRobotArmInRestingPose() const
{
  const auto action_name = "getRobotArmInRestingPose";
  auto move_group_ptr = buildMoveItGroupHandle();
  const robot_state::JointModelGroup* joint_model_group =
      move_group_ptr->getCurrentState()->getJointModelGroup(
          robot_specific_interface_->getRobotPlanningGroup());
  {
    INFO("{}: {} is calculating the target", action_name, getRobotName());
    moveit::core::RobotStatePtr current_state = move_group_ptr->getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    robot_specific_interface_->patchJointStateValuesForArmInRestingPose(joint_group_positions);
    move_group_ptr->setJointValueTarget(joint_group_positions);
    move_group_ptr->setStartState(*move_group_ptr->getCurrentState());
  }
  moveit::planning_interface::MoveGroupInterface::Plan movement_plan;
  {
    INFO("{}: {} is planning", action_name, getRobotName());
    auto success = (move_group_ptr->plan(movement_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
    {
      ERROR("{}: {} failed to generate a plan", action_name, getRobotName());
      return false;
    }
  }
  {
    INFO("{}: {} is executing", action_name, getRobotName());
    auto success = (move_group_ptr->execute(movement_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
    {
      ERROR("{}: {} failed to execute", action_name, getRobotName());
      return false;
    }
  }
  INFO("{}: {} finished execution", action_name, getRobotName());
  return true;
}

bool PickAndPlaceRobotMovements::getGripperIn3DPoseJoinSpace(
    const tijmath::RelativePose3& target) const
{
  const auto action_name = "getGripperIn3DPoseJoinSpace";

  if (!getRobotHealthState())
  {
    return false;
  }

  auto move_group_ptr = buildMoveItGroupHandle();
  move_group_ptr->setPoseReferenceFrame(world_frame);

  INFO("{}: {} target gripper pose at {}", action_name, getRobotName(), target);

  moveit::planning_interface::MoveGroupInterface::Plan movement_plan;
  {
    INFO("{}: {} is generating the moveit plan", action_name, getRobotName());
    auto target_geo_pose = utils::convertCorePoseToGeoPose(target.pose());
    move_group_ptr->setPoseTarget(target_geo_pose);
    move_group_ptr->setStartState(*move_group_ptr->getCurrentState());

    bool success = (move_group_ptr->plan(movement_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
    {
      ERROR("{}: {} failed to generate a plan", action_name, getRobotName());
      return false;
    }
  }

  {
    INFO("{}: {} is executing the plan", action_name, getRobotName());
    auto success = (move_group_ptr->execute(movement_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
    {
      ERROR("{}: {} failed to execute the plan", action_name, getRobotName());
      return false;
    }
  }

  INFO("{}: {} completed the execution of the plan", action_name, getRobotName());
  return true;
}

bool PickAndPlaceRobotMovements::getRobotTo2DPose(const tijmath::RelativePose3& target) const
{
  const auto action_name = "getRobotTo2DPose";
  if (!getRobotHealthState())
  {
    INFO("{}: {} failed to execute because the robot is disabled", action_name, getRobotName());
    return false;
  }
  auto move_group_ptr = buildMoveItGroupHandle();
  const robot_state::JointModelGroup* joint_model_group =
      move_group_ptr->getCurrentState()->getJointModelGroup(
          robot_specific_interface_->getRobotPlanningGroup());
  {
    INFO("{}: {} is calculating the target", action_name, getRobotName());
    moveit::core::RobotStatePtr current_state = move_group_ptr->getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group,

                                           joint_group_positions);
    robot_specific_interface_->patchJointStateValuesForArmInRestingPose(joint_group_positions);
    robot_specific_interface_->patchJointStateValuesToGoTo2DPose(joint_group_positions, target);

    move_group_ptr->setJointValueTarget(joint_group_positions);
    move_group_ptr->setStartState(*move_group_ptr->getCurrentState());
  }
  moveit::planning_interface::MoveGroupInterface::Plan movement_plan;
  {
    INFO("{}: {} is planning", action_name, getRobotName());
    auto success = (move_group_ptr->plan(movement_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
    {
      ERROR("{}: {} failed to generate a plan", action_name, getRobotName());
      return false;
    }
  }
  {
    INFO("{}: {} is executing", action_name, getRobotName());
    auto success = (move_group_ptr->execute(movement_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
    {
      ERROR("{}: {} failed to execute", action_name, getRobotName());
      return false;
    }
  }
  INFO("{}: {} finished execution", action_name, getRobotName());
  return true;
}

bool PickAndPlaceRobotMovements::getRobotInSafePoseNearTarget(
    const tijmath::RelativePose3& target) const
{
  const auto action_name = "getRobotInSafePoseNearTarget";
  if (!getRobotHealthState())
  {
    INFO("{}: {} failed to execute because the robot is disabled", action_name, getRobotName());
    return false;
  }
  auto move_group_ptr = buildMoveItGroupHandle();
  const robot_state::JointModelGroup* joint_model_group =
      move_group_ptr->getCurrentState()->getJointModelGroup(
          robot_specific_interface_->getRobotPlanningGroup());
  {
    INFO("{}: {} is calculating the target", action_name, getRobotName());
    moveit::core::RobotStatePtr current_state = move_group_ptr->getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group,

                                           joint_group_positions);
    robot_specific_interface_->patchJointStateValuesForArmInRestingPose(joint_group_positions);
    robot_specific_interface_->patchJointStateValuesToGetCloseToTargetPose(joint_group_positions,
                                                                           target);

    move_group_ptr->setJointValueTarget(joint_group_positions);
    move_group_ptr->setStartState(*move_group_ptr->getCurrentState());
  }
  moveit::planning_interface::MoveGroupInterface::Plan movement_plan;
  {
    INFO("{}: {} is planning", action_name, getRobotName());
    auto success = (move_group_ptr->plan(movement_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
    {
      ERROR("{}: {} failed to generate a plan", action_name, getRobotName());
      return false;
    }
  }
  {
    INFO("{}: {} is executing", action_name, getRobotName());
    auto success = (move_group_ptr->execute(movement_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
    {
      ERROR("{}: {} failed to execute", action_name, getRobotName());
      return false;
    }
  }
  INFO("{}: {} finished execution", action_name, getRobotName());
  return true;
}

bool PickAndPlaceRobotMovements::contactPartFromAboveAndGrasp(
    const tijmath::RelativePose3& target_end_effector_pose) const
{
  const auto action_name = "contactPartFromAboveAndGrasp";
  if (!getRobotHealthState())
  {
    INFO("{}: {} failed to execute because the robot is disabled", action_name, getRobotName());
    return false;
  }
  auto move_group_ptr = buildMoveItGroupHandle();

  INFO("{}: {} is calculating the approximation trajectory", action_name, getRobotName());
  // configure tighter tolerances for this
  useNarrowTolerances(true);
  // make sure we trade in world poses only
  move_group_ptr->setPoseReferenceFrame(world_frame);
  // turn on the gripper
  setRobotGripperState(true);

  auto frame_transformer = toolbox_->getFrameTransformer();
  auto target_end_effector_pose_in_world =
      frame_transformer->transformPoseToFrame(target_end_effector_pose, world_frame);

  // notice that part poses get detected with a height equal to about half the
  // height of the piece
  const auto run_top =
      target_end_effector_pose_in_world.position().vector().z() + pick_search_length_ * 0.5;
  const auto run_bottom =
      target_end_effector_pose_in_world.position().vector().z() - pick_search_length_ * 0.5;
  target_end_effector_pose_in_world.position().vector().z() = run_top;

  while (!getRobotGripperAttachementState() &&
         (target_end_effector_pose_in_world.position().vector().z() > run_bottom))
  {
    // decrease the gripper one step
    target_end_effector_pose_in_world.position().vector().z() -= pickup_displacement_step_;

    INFO("{}: {}  is generating the step moveit plan", action_name, getRobotName());
    moveit::planning_interface::MoveGroupInterface::Plan movement_plan;
    {
      auto target_geo_pose_in_world =
          utils::convertCorePoseToGeoPose(target_end_effector_pose_in_world.pose());
      move_group_ptr->setPoseTarget(target_geo_pose_in_world);
      move_group_ptr->setStartState(*move_group_ptr->getCurrentState());

      bool success = (move_group_ptr->plan(movement_plan) ==
                      moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (!success)
      {
        ERROR("{}: {} failed to generate a plan for step, aborting grasping", action_name,
              getRobotName());
        return false;
      }
    }

    INFO("{}: {} is executing the step moveit plan", action_name, getRobotName());
    move_group_ptr->execute(movement_plan);
  }

  if (getRobotGripperAttachementState())
  {
    INFO("{}: {} succeeded to grasp the part", action_name, getRobotName());
    return true;
  }

  INFO("{}: {} failed to grasp the part", action_name, getRobotName());
  return true;
}

bool PickAndPlaceRobotMovements::getGripperIn3DPoseCartesianSpace(
    const tijmath::RelativePose3& target) const
{
  const auto action_name = "getGripperIn3DPoseCartesianSpace";
  if (!getRobotHealthState())
  {
    INFO("{}: {} failed to execute because the robot is disabled", action_name, getRobotName());
    return false;
  }
  auto move_group_ptr = buildMoveItGroupHandle();
  move_group_ptr->setPoseReferenceFrame(world_frame);

  INFO("Place movement: {} is calculating the pickup trajectory", getRobotName());

  auto frame_transformer = toolbox_->getFrameTransformer();

  const auto target_in_world_pose = frame_transformer->transformPoseToFrame(target, world_frame);

  {
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(utils::convertCorePoseToGeoPose(target_in_world_pose.pose()));

    INFO("Place will drop the part once we are at {} in {} frame",
         utils::convertGeoPoseToCorePose(waypoints.at(0)), world_frame);

    INFO("Place movement: {} is generating the moveit plan", getRobotName());
    moveit_msgs::RobotTrajectory trajectory;
    {
      const double fraction = move_group_ptr->computeCartesianPath(
          waypoints, pickup_displacement_step_, pickup_displacement_jump_threshold_, trajectory);
      DEBUG("Place movement: cartesian planner plan fraction for {}: {}", getRobotName(), fraction);
      bool success = (fraction > 0.95);
      if (!success)
      {
        ERROR(
            "{} failed to generate a plan to the drop point "
            "(fraction: {})",
            getRobotName(), fraction);
        WARNING("{} will release the part here");
        setRobotGripperState(false);
        return false;
      }
    }

    // we don't check the return value because it's going to return failure once
    // we hit the table
    INFO("Place movement: {} is executing the movement", getRobotName());
    move_group_ptr->execute(trajectory);
  }

  setRobotGripperState(false);
  INFO("{} placed the part that the destination", getRobotName());
  return true;
}

bool PickAndPlaceRobotMovements::getRobotGripperOn() const
{
  const auto action_name = "getRobotGripperOn";
  INFO("{}: {} is turning on the gripper", action_name, getRobotName());
  setRobotGripperState(true);
  return true;
}

bool PickAndPlaceRobotMovements::getRobotGripperOff() const
{
  const auto action_name = "getRobotGripperOff";
  INFO("{}: {} is turning off the gripper", action_name, getRobotName());
  setRobotGripperState(false);
  return true;
}

bool PickAndPlaceRobotMovements::setRobotGripperToolType(
    const tijcore::GripperTypeId new_type) const
{
  const auto action_name = "switchToolType";
  INFO("{}: {} is changing the tool type to {}", action_name, getRobotName(), new_type);
  return robot_specific_interface_->setGripperToolTypeImpl(new_type);
}

tijcore::GripperTypeId PickAndPlaceRobotMovements::getRobotGripperToolType() const
{
  return robot_specific_interface_->getGripperToolTypeImpl();
}

void PickAndPlaceRobotMovements::abortCurrentAction() const
{
  const auto action_name = "abortCurrentAction";
  auto move_group_ptr = buildMoveItGroupHandle();
  WARNING("{}: {} is aborting", action_name, getRobotName());
  move_group_ptr->stop();
}

void PickAndPlaceRobotMovements::buildObstacleSceneFromDescription() const
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
                                                   z_offset, 0.0, 0.0, 0.0, operation));
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
                                                   z_offset, 0.0, 0.0, 0.0, operation));

    collision_objects.push_back(createCollisionBox(item.name, "wall_1", item.frame_id, 0.64, 0.06,
                                                   0.08, 0.0, 0.31, 0.04, operation));
    collision_objects.push_back(createCollisionBox(item.name, "wall_2", item.frame_id, 0.64, 0.06,
                                                   0.08, 0.0, -0.31, 0.04, operation));
    collision_objects.push_back(createCollisionBox(item.name, "wall_3", item.frame_id, 0.06, 0.64,
                                                   0.08, 0.31, 0.0, 0.04, operation));
    collision_objects.push_back(createCollisionBox(item.name, "wall_4", item.frame_id, 0.06, 0.64,
                                                   0.08, -0.31, 0.0, 0.04, operation));
  }

  DEBUG(" - adding table representatives");
  for (const auto& item : scene_configuration->getListOfTables())
  {
    collision_objects.push_back(createCollisionBox(item.name, "envelope", item.frame_id, 0.9, 1.5,
                                                   1.0, 0.0, 0.0, 0.95, operation));
  }

  DEBUG(" - adding conveyor belt representatives");
  for (const auto& item : scene_configuration->getListOfConveyorBelts())
  {
    collision_objects.push_back(createCollisionBox(item.name, "surface", item.container_frame_id,
                                                   0.63, 9.0, z_offset, 0.0, 0.0, 0.0, operation));
  }

  DEBUG(" - adding static obstacles");

  // kitting rail
  collision_objects.push_back(createCollisionBox("static", "kittingrail", "world", 0.4, 10.0, 0.10,
                                                 -1.3, 0.0, 0.93, operation));

  // Imaginary divider
  collision_objects.push_back(
      createCollisionBox("static", "divider", "world", 0.05, 10.0, 0.1, -1.4, 0.0, 2.7, operation));

  // tool-swapping table
  collision_objects.push_back(createCollisionBox("static", "toolswappingtable", "world", 1.0, 1.5,
                                                 1.0, -3.7, 6.26, 0.5, operation));

  planning_scene_ptr_->applyCollisionObjects(collision_objects);
}

void PickAndPlaceRobotMovements::alignEndEffectorWithTarget(
    tijmath::RelativePose3& end_effector_target_pose) const
{
  const auto orientation = end_effector_target_pose.rotation().rotationMatrix();
  const auto x_director = tijmath::Vector3{ 0.0, 0.0, -1 };

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
    // unfeasible plans. It's likely related to the end of range of the wrist
    // articulations.
    z_director = orientation.col(2) * (-1);
  }
  else
  {
    // is horizontal, always choose x
    // TODO(glpuga) the sign inversion is because without this accessing the
    // briefcases to remove a battery fails because the full body of the robot
    // is oriented to try to match the orientation of the piece, resulting in
    // unfeasible plans. It's likely related to the end of range of the wrist
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

tijmath::RelativePose3 PickAndPlaceRobotMovements::calculateVerticalLandingPose(
    const tijmath::RelativePose3& target, const double offset_to_top) const
{
  auto frame_transformer = toolbox_->getFrameTransformer();
  auto target_pose_in_world = frame_transformer->transformPoseToFrame(target, world_frame);

  auto vertical_approximation_pose =
      calculateVerticalGripEndEffectorPose(target_pose_in_world, offset_to_top);
  vertical_approximation_pose.position().vector().z() += landing_pose_height_;

  return vertical_approximation_pose;
}

tijmath::RelativePose3 PickAndPlaceRobotMovements::calculateVerticalDropPose(
    const tijmath::RelativePose3& target, const double offset_to_top) const
{
  auto frame_transformer = toolbox_->getFrameTransformer();
  auto target_pose_in_world = frame_transformer->transformPoseToFrame(target, world_frame);

  auto drop_pose_in_world =
      calculateVerticalGripEndEffectorPose(target_pose_in_world, offset_to_top);
  drop_pose_in_world.position().vector().z() += part_drop_height_;

  // hacky way to compensate for the offset being about half the height
  drop_pose_in_world.position().vector().z() += offset_to_top;

  return drop_pose_in_world;
}

tijmath::RelativePose3 PickAndPlaceRobotMovements::calculateVerticalGripEndEffectorPose(
    const tijmath::RelativePose3& target, const double offset_to_top) const
{
  auto frame_transformer = toolbox_->getFrameTransformer();
  auto end_effector_pose = frame_transformer->transformPoseToFrame(target, world_frame);

  alignEndEffectorWithTarget(end_effector_pose);
  end_effector_pose.position().vector().z() += offset_to_top;

  // TODO(glpuga) Issue https://github.com/glpuga/ariac2021ws/issues/258
  // The following is a hack to compensate for the fact that the end effector of
  // kitting seems to be different from the one of gantry.
  if (robot_specific_interface_->getRobotName() == "kitting")
  {
    end_effector_pose.position().vector().z() += 0.02;
  }

  return end_effector_pose;
}

bool PickAndPlaceRobotMovements::getRobotGripperAttachementState() const
{
  return robot_specific_interface_->getRobotGripperAttachementState();
}

bool PickAndPlaceRobotMovements::getRobotHealthState() const
{
  return robot_specific_interface_->getRobotHealthState();
}

std::string PickAndPlaceRobotMovements::getRobotName() const
{
  return robot_specific_interface_->getRobotName();
}

bool PickAndPlaceRobotMovements::testIfRobotReachesPose(const tijmath::RelativePose3& target) const
{
  return robot_specific_interface_->testIfRobotReachesPose(target);
}

void PickAndPlaceRobotMovements::setRobotGripperState(const bool state) const
{
  robot_specific_interface_->setRobotGripperState(state);
}

}  // namespace tijros
