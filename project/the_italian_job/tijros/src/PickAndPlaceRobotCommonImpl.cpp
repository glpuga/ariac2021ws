/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// Standard library
#include <chrono>
#include <future>
#include <string>
#include <thread>

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
#include <logger/logger.hpp>
#include <tijcore/utils/angles.hpp>
#include <tijros/PickAndPlaceRobotCommonImpl.hpp>
#include <tijros/utils/utils.hpp>

namespace tijros {

namespace {

// TODO(glpuga) this frame id is hardcoded in multiple places. Use the one that
// can be get through the scene configuration in  the toolbox
static char world_frame[] = "world";

static const double landing_pose_height = 0.25;
static const double pick_search_length = 0.20;
// TODO(glpuga) this should be 10, but needs to be higher since cartesian
// placement seems to ignore nearby objects
static const double place_drop_height = 0.11;

static const double pickup_displacement_jump_threshold = 10.0;
static const double pickup_displacement_step = 0.0025;
static const double pickup_displacement_step_div = 15;
static const double max_planning_time = 90.0;
static const int max_planning_attempts = 5;
static const double goal_position_tolerance = 0.05;
static const double goal_orientation_tolerance =
    tijcore::utils::angles::degreesToRadians(2.5);

moveit_msgs::CollisionObject
createCollisionBox(const std::string &id, const std::string &sub_id,
                   const std::string &frame_id, const double width,
                   const double height, const double depth,
                   const double xoffset = 0.0, const double yoffset = 0.0,
                   const double zoffset = 0.0,
                   const int operation = moveit_msgs::CollisionObject::ADD) {
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

} // namespace

PickAndPlaceRobotCommonImpl::PickAndPlaceRobotCommonImpl(
    const tijcore::Toolbox::SharedPtr &toolbox)
    : toolbox_{toolbox} {}

moveit::planning_interface::MoveGroupInterface *
PickAndPlaceRobotCommonImpl::getMoveItGroupHandlePtr() const {
  // if the setup had not been performed before, do it now
  // Ugly AF, need to find a better solution.

  // TODO(glpuga) for some reason, keeping state in this object causes
  // errors in one of the joints, in both robots(!)
  const auto base_namespace = "/ariac/" + name();

  move_group_ptr_.reset();
  if (!move_group_ptr_) {

    moveit::planning_interface::MoveGroupInterface::Options options{
        getRobotPlanningGroup(), base_namespace + "/robot_description",
        ros::NodeHandle(base_namespace)};
    move_group_ptr_ =
        std::make_unique<moveit::planning_interface::MoveGroupInterface>(
            options);
    move_group_ptr_->setPlanningTime(max_planning_time);
    move_group_ptr_->setNumPlanningAttempts(max_planning_attempts);
    move_group_ptr_->setGoalPositionTolerance(goal_position_tolerance);
    move_group_ptr_->setGoalOrientationTolerance(goal_orientation_tolerance);
  }

  if (!planning_scene_ptr_) {
    planning_scene_ptr_ =
        std::make_unique<moveit::planning_interface::PlanningSceneInterface>(
            base_namespace);
    setupObjectConstraints();
  }

  return move_group_ptr_.get();
}

bool PickAndPlaceRobotCommonImpl::getInSafePose() const {
  auto move_group_ptr = getMoveItGroupHandlePtr();

  const robot_state::JointModelGroup *joint_model_group =
      move_group_ptr->getCurrentState()->getJointModelGroup(
          getRobotPlanningGroup());

  INFO("Get-in-rest-pose movement: {} is calcularting the target", name());
  {
    moveit::core::RobotStatePtr current_state =
        move_group_ptr->getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group,

                                           joint_group_positions);
    patchJointStateValuesForRestingPose(joint_group_positions);
    move_group_ptr->setJointValueTarget(joint_group_positions);
    move_group_ptr->setStartState(*move_group_ptr->getCurrentState());
  }

  INFO("Get-in-rest-pose movement: {} is planning", name());
  moveit::planning_interface::MoveGroupInterface::Plan movement_plan;
  {
    auto success = (move_group_ptr->plan(movement_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success) {
      ERROR("{} failed to generate a plan", name());
      return false;
    }
  }

  INFO("Get-in-rest-pose movement: {} is executing", name());
  {
    auto success = (move_group_ptr->execute(movement_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success) {
      ERROR("{} failed to execute", name());
      return false;
    }
  }

  INFO("Get-in-rest-pose movement: {} finished execution", name());
  return true;
}

bool PickAndPlaceRobotCommonImpl::getInSafePoseNearTarget(
    const tijcore::RelativePose3 &target) const {
  if (!enabled()) {
    return false;
  }
  auto move_group_ptr = getMoveItGroupHandlePtr();

  const robot_state::JointModelGroup *joint_model_group =
      move_group_ptr->getCurrentState()->getJointModelGroup(
          getRobotPlanningGroup());

  INFO("Get-in-rest-pose movement: {} is calcularting the target", name());
  {
    moveit::core::RobotStatePtr current_state =
        move_group_ptr->getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group,

                                           joint_group_positions);
    patchJointStateValuesForRestingPose(joint_group_positions);
    patchJointStateValuesToGetCloseToTarget(joint_group_positions, target);

    move_group_ptr->setJointValueTarget(joint_group_positions);

    move_group_ptr->setStartState(*move_group_ptr->getCurrentState());
  }

  INFO("Get-in-rest-pose movement: {} is planning", name());
  moveit::planning_interface::MoveGroupInterface::Plan movement_plan;
  {
    auto success = (move_group_ptr->plan(movement_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success) {
      ERROR("{} failed to generate a plan", name());
      return false;
    }
  }

  INFO("Get-in-rest-pose movement: {} is executing", name());
  {
    auto success = (move_group_ptr->execute(movement_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success) {
      ERROR("{} failed to execute", name());
      return false;
    }
  }

  INFO("Get-in-rest-pose movement: {} finished execution", name());
  return true;
}

bool PickAndPlaceRobotCommonImpl::getToGraspingPoseHint(
    const tijcore::RelativePose3 &target) const {
  if (!enabled()) {
    return false;
  }
  auto move_group_ptr = getMoveItGroupHandlePtr();

  const robot_state::JointModelGroup *joint_model_group =
      move_group_ptr->getCurrentState()->getJointModelGroup(
          getRobotPlanningGroup());

  INFO("Closest-safe-spot movement: {} is calcularting the target joint for "
       "resting pose",
       name());
  {
    moveit::core::RobotStatePtr current_state =
        move_group_ptr->getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group,
                                           joint_group_positions);

    patchJointStateValuesForRestingPose(joint_group_positions);
    patchJointStateValuesGraspingHingPoseNearTarget(joint_group_positions,
                                                    target);

    move_group_ptr->setJointValueTarget(joint_group_positions);

    move_group_ptr->setStartState(*move_group_ptr->getCurrentState());
  }

  INFO("Closest-safe-spot movement: {} is planning", name());
  moveit::planning_interface::MoveGroupInterface::Plan movement_plan;
  {
    auto success = (move_group_ptr->plan(movement_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success) {
      ERROR("{} failed to generate a plan", name());
      return false;
    }
  }

  INFO("Closest-safe-spot movement: {} is executing", name());
  {
    auto success = (move_group_ptr->execute(movement_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success) {
      ERROR("{} failed to move execute", name());
      return false;
    }
  }

  INFO("Closest-safe-spot movement: {} plan complete", name());
  return true;
}

bool PickAndPlaceRobotCommonImpl::getInLandingSpot(
    const tijcore::RelativePose3 &target) const {
  if (!enabled()) {
    return false;
  }
  auto move_group_ptr = getMoveItGroupHandlePtr();

  // Convert the target pose to the world reference frame, and set the
  // orientation pointing to the part
  auto frame_transformer = toolbox_->getFrameTransformer();
  auto end_effector_target_pose =
      frame_transformer->transformPoseToFrame(target, world_frame);
  alignEndEffectorWithTarget(end_effector_target_pose);

  INFO("Approximation movement: {} is calculating the approximation pose",
       name());
  auto approximation_pose_in_world = end_effector_target_pose;
  // add the approximation height, assumes now zunit points up in world.
  approximation_pose_in_world.position().vector().z() += landing_pose_height;

  DEBUG("Target in original frame at {}", target);
  DEBUG("Target in world frame at {}", end_effector_target_pose);
  INFO("Approximation pose at {}: {} ", world_frame,
       approximation_pose_in_world);

  INFO("Approximation movement: {} is generating the moveit plan", name());
  moveit::planning_interface::MoveGroupInterface::Plan movement_plan;
  {
    auto target_geo_pose =
        utils::convertCorePoseToGeoPose(approximation_pose_in_world.pose());
    move_group_ptr->setPoseTarget(target_geo_pose);
    move_group_ptr->setPoseReferenceFrame(
        approximation_pose_in_world.frameId());

    move_group_ptr->setStartState(*move_group_ptr->getCurrentState());

    bool success = (move_group_ptr->plan(movement_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success) {
      ERROR("{} failed to generate a plan to arrive at the approximation pose",
            name());
      return false;
    }
  }

  INFO("Approximation movement: {} is executing the movement", name());
  {
    auto success = (move_group_ptr->execute(movement_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success) {
      ERROR("{} failed to move arm into the approximation pose", name());
      return false;
    }
  }

  INFO("{} arrived at the approximation pose for ", name());
  return true;
}

bool PickAndPlaceRobotCommonImpl::graspPartFromAbove(
    const tijcore::RelativePose3 &target) const {
  if (!enabled()) {
    return false;
  }
  auto move_group_ptr = getMoveItGroupHandlePtr();

  INFO("Pick-up movement: {} is calculating the pickup trajectory", name());

  // make sure we trade in world poses only
  move_group_ptr->setPoseReferenceFrame(world_frame);
  setSuctionGripper(true);

  auto frame_transformer = toolbox_->getFrameTransformer();
  auto end_effector_target_pose =
      frame_transformer->transformPoseToFrame(target, world_frame);
  alignEndEffectorWithTarget(end_effector_target_pose);
  auto end_effector_target_pose_in_world = end_effector_target_pose.pose();

  const auto run_top =
      end_effector_target_pose_in_world.position().vector().z() +
      pick_search_length * 0.5;
  const auto run_bottom =
      end_effector_target_pose_in_world.position().vector().z() -
      pick_search_length * 0.5;
  end_effector_target_pose_in_world.position().vector().z() = run_top;

  while (!gripperHasPartAttached() &&
         (end_effector_target_pose_in_world.position().vector().z() >
          run_bottom)) {
    std::vector<geometry_msgs::Pose> waypoints;

    // make sure we are pointing down in the end pose
    end_effector_target_pose_in_world.position().vector().z() -=
        pickup_displacement_step;
    waypoints.push_back(
        utils::convertCorePoseToGeoPose(end_effector_target_pose_in_world));

    INFO("Pick-up trajectory going to pose {} in {} frame",
         utils::convertGeoPoseToCorePose(waypoints.at(0)), world_frame);

    INFO("Pick-up movement: {} is generating the moveit plan", name());
    moveit_msgs::RobotTrajectory trajectory;
    {
      const double fraction = move_group_ptr->computeCartesianPath(
          waypoints, pickup_displacement_step / pickup_displacement_step_div,
          pickup_displacement_jump_threshold, trajectory);
      DEBUG("Pick-up movement: cartesian planner plan fraction for {}: {}",
            name(), fraction);
      bool success = (fraction > 0.8);
      if (!success) {
        ERROR("{} failed to generate a plan for the pick up trajectory "
              "(fraction: {})",
              name(), fraction);
        break;
      }
    }

    INFO("Pick-up movement: {} is executing the movement", name());
    move_group_ptr->execute(trajectory);
  }

  if (gripperHasPartAttached()) {
    INFO("{} successfully grasped a part!", name());
    return true;
  }

  ERROR("{} failed to pick a part", name());
  return true;
}

bool PickAndPlaceRobotCommonImpl::placePartFromAbove(
    const tijcore::RelativePose3 &target) const {
  if (!enabled()) {
    return false;
  }
  auto move_group_ptr = getMoveItGroupHandlePtr();

  INFO("Place movement: {} is calculating the pickup trajectory", name());

  // make sure we trade in world poses only
  move_group_ptr->setPoseReferenceFrame(world_frame);

  auto frame_transformer = toolbox_->getFrameTransformer();
  auto end_effector_target_pose =
      frame_transformer->transformPoseToFrame(target, world_frame);
  alignEndEffectorWithTarget(end_effector_target_pose);

  auto end_effector_target_pose_in_world = end_effector_target_pose.pose();

  end_effector_target_pose_in_world.position().vector().z() +=
      place_drop_height;

  {
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(
        utils::convertCorePoseToGeoPose(end_effector_target_pose_in_world));

    INFO("Place will drop the part once we are at {} in {} frame",
         utils::convertGeoPoseToCorePose(waypoints.at(0)), world_frame);

    INFO("Place movement: {} is generating the moveit plan", name());
    moveit_msgs::RobotTrajectory trajectory;
    {
      const double fraction = move_group_ptr->computeCartesianPath(
          waypoints, pickup_displacement_step / pickup_displacement_step_div,
          pickup_displacement_jump_threshold, trajectory);
      DEBUG("Place movement: cartesian planner plan fraction for {}: {}",
            name(), fraction);
      bool success = (fraction > 0.8);
      if (!success) {
        ERROR("{} failed to generate a plan to the drop point "
              "(fraction: {})",
              name(), fraction);
        WARNING("{} will release the part here");
        setSuctionGripper(false);
        return false;
      }
    }

    INFO("Place movement: {} is executing the movement", name());
    auto success = (move_group_ptr->execute(trajectory) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success) {
      ERROR("{} failed to move arm to the drop point.", name());
      WARNING("{} will release the part here");
      setSuctionGripper(false);
      return false;
    }
  }

  setSuctionGripper(false);
  INFO("{} placed the part that the destination", name());
  return true;
}

bool PickAndPlaceRobotCommonImpl::dropPartWhereYouStand() const {
  setSuctionGripper(false);
  INFO("{} turned the suction gripper off", name());
  return true;
}

void PickAndPlaceRobotCommonImpl::cancelAction() {
  // TODO(glpuga) This will is a half-assed solution to this,
  // and will probably only work "most" of the time.
  // move_group_ptr->stop();
}

void PickAndPlaceRobotCommonImpl::setupObjectConstraints() const {
  std::vector<moveit_msgs::CollisionObject> collision_objects;

  auto scene_configuration = toolbox_->getSceneConfigReader();
  const double z_offset{0.025};

  INFO(" Generating collision scene");
  INFO(" - adding AGV tray representatives");
  for (const auto &item : scene_configuration->getListOfAgvs()) {
    collision_objects.push_back(createCollisionBox(
        item.name, "surface", item.frame_id, 0.5, 0.7, z_offset));
    collision_objects.push_back(createCollisionBox(
        item.name, "tower", item.frame_id, 0.25, 0.25, 0.8, 0.0, -0.45, 0.4));
  }

  INFO(" - adding assembly station representatives");
  for (const auto &item : scene_configuration->getListOfAssemblyStations()) {
    collision_objects.push_back(createCollisionBox(item.name, "briefcase_table",
                                                   item.frame_id, 1.6, 1.2,
                                                   z_offset, -0.3, -0.1, 0.0));
    collision_objects.push_back(createCollisionBox(item.name, "briefcase_front",
                                                   item.frame_id, 0.05, 0.6,
                                                   0.16, 0.3, 0.0, 0.08));
    collision_objects.push_back(createCollisionBox(item.name, "briefcase_side",
                                                   item.frame_id, 0.6, 0.05,
                                                   0.16, 0.0, -0.3, 0.08));
    collision_objects.push_back(createCollisionBox(item.name, "table_right",
                                                   item.frame_id, 1.0, 0.10,
                                                   1.0, -0.70, -0.75, 0.5));
    collision_objects.push_back(createCollisionBox(item.name, "table_back",
                                                   item.frame_id, 0.8, 1.2, 1.0,
                                                   -0.7, -0.1, 0.5));
    collision_objects.push_back(createCollisionBox(
        item.name, "table_left", item.frame_id, 1.6, 0.1, 1.0, -0.45, 0.5, 0.5));
    collision_objects.push_back(createCollisionBox(
        item.name, "briefcase_top_1", item.frame_id, 0.1, 0.1, 0.80, 0.3, 0.35, 0.40));
    collision_objects.push_back(createCollisionBox(
        item.name, "briefcase_top_2", item.frame_id, 0.7, 0.1, 0.1, 0.0, 0.35, 0.75));
  }

  INFO(" - adding bin representatives");
  for (const auto &item : scene_configuration->getListOfBins()) {
    collision_objects.push_back(createCollisionBox(
        item.name, "surface", item.frame_id, 0.6, 0.6, z_offset));

    collision_objects.push_back(createCollisionBox(
        item.name, "wall_1", item.frame_id, 0.64, 0.06, 0.08, 0.0, 0.31, 0.04));
    collision_objects.push_back(createCollisionBox(item.name, "wall_2",
                                                   item.frame_id, 0.64, 0.06,
                                                   0.08, 0.0, -0.31, 0.04));
    collision_objects.push_back(createCollisionBox(
        item.name, "wall_3", item.frame_id, 0.06, 0.64, 0.08, 0.31, 0.0, 0.04));
    collision_objects.push_back(createCollisionBox(item.name, "wall_4",
                                                   item.frame_id, 0.06, 0.64,
                                                   0.08, -0.31, 0.0, 0.04));
  }

  // kitting rail
  collision_objects.push_back(createCollisionBox(
      "kitting", "rail", "world", 0.4, 10.0, 0.10, -1.3, 0.0, 0.93));

  // Imaginary divider
  collision_objects.push_back(createCollisionBox(
      "divider", "rail", "world", 0.05, 10.0, 0.1, -1.4, 0.0, 2.7));

  // conveyor belt
  collision_objects.push_back(createCollisionBox(
      "kitting", "belt", "world", 0.8, 9.0, 0.2, -0.5, 0.0, 0.9));

  planning_scene_ptr_->applyCollisionObjects(collision_objects);
}

void PickAndPlaceRobotCommonImpl::markAsCommanded(
    const std::vector<tijcore::ModelTraySharedAccessSpaceDescription>
        &descriptors,
    const int command) {

  // TODO(glpuga) improve this
  // this method call is needed to generate the pointers if planning_scene_ptr_
  // has not been populated yet.
  getMoveItGroupHandlePtr();

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  for (const auto &item : descriptors) {
    const auto &frame_id = item.center.frameId();
    const auto &offsets = item.center.position().vector();
    collision_objects.push_back(createCollisionBox(
        "access", item.id, frame_id, item.x_size, item.y_size, item.z_size,
        offsets.x(), offsets.y(), offsets.z(), command));
  }
  planning_scene_ptr_->applyCollisionObjects(collision_objects);
}

void PickAndPlaceRobotCommonImpl::markAsInaccessible(
    const std::vector<tijcore::ModelTraySharedAccessSpaceDescription>
        &descriptors) {
  markAsCommanded(descriptors, moveit_msgs::CollisionObject::ADD);
}

void PickAndPlaceRobotCommonImpl::markAsAccessible(
    const std::vector<tijcore::ModelTraySharedAccessSpaceDescription>
        &descriptors) {
  markAsCommanded(descriptors, moveit_msgs::CollisionObject::REMOVE);
}

void PickAndPlaceRobotCommonImpl::alignEndEffectorWithTarget(
    tijcore::RelativePose3 &end_effector_target_pose) const {
  const auto orientation = end_effector_target_pose.rotation().rotationMatrix();
  const auto x_director = tijcore::Vector3{0, 0, -1};

  // I add both the x and y axes, because if the part is on the side, one of
  // them might be aligned with the Z axis of the world frame.
  auto y_director = orientation.col(0) + orientation.col(1);
  // don't assume the target will have it's axes normal to the Z axis
  y_director -= x_director * y_director.dot(x_director);
  y_director = y_director / y_director.norm();
  auto z_director = x_director.cross(y_director);
  z_director = z_director / z_director.norm();
  const auto end_effector_orientation =
      tijcore::Matrix3{x_director, y_director, z_director}.trans();
  end_effector_target_pose.rotation() =
      tijcore::Rotation{end_effector_orientation};
}

} // namespace tijros
