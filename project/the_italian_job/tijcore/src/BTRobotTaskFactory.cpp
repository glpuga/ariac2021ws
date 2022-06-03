/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <chrono>
#include <memory>
#include <string>
#include <utility>

// external
#include <behavior_tree_extras/BehaviorTreeBuilder.hpp>

// tijcore
#include <tijcore/tasking/BTRobotTaskFactory.hpp>
#include <tijcore/tasking/BTTaskParameters.hpp>
#include <tijcore/tasking/BehaviorTreeWrappedTask.hpp>
#include <tijcore/tasking/PickAndPlaceTask.hpp>
#include <tijcore/tasking/RemoveBrokenPartTask.hpp>
#include <tijcore/tasking/SubmitAssemblyShipmentTask.hpp>
#include <tijcore/tasking/SubmitKittingShipmentTask.hpp>
#include <tijcore/utils/BehaviorTreeTransitionLogger.hpp>
#include <tijlogger/logger.hpp>

// tijcore nodes
#include <tijcore/btnodes/AbortCurrentActionNode.hpp>
#include <tijcore/btnodes/CalculateEndEffectorPoseFromPayloadPoseNode.hpp>
#include <tijcore/btnodes/CalculateEnvelopeAndOffsetForVerticalPickUpNode.hpp>
#include <tijcore/btnodes/CalculatePayloadIntoEndEffectorTransformFromCamerasIfPossibleNode.hpp>
#include <tijcore/btnodes/CalculatePayloadIntoEndEffectorTransformNode.hpp>
#include <tijcore/btnodes/CalculateRegulatorPreInsertAndInsertPosesNode.hpp>
#include <tijcore/btnodes/CalculateSensorPreInsertAndInsertPosesNode.hpp>
#include <tijcore/btnodes/CalculateVerticalDropPoseNode.hpp>
#include <tijcore/btnodes/CalculateVerticalGripEndEffectorPoseNode.hpp>
#include <tijcore/btnodes/CalculateVerticalLandingPoseNode.hpp>
#include <tijcore/btnodes/CalibrateSourcePartFromPerceptionIfPossibleNode.hpp>
#include <tijcore/btnodes/ContactPartFromAboveAndGraspNode.hpp>
#include <tijcore/btnodes/CreateEmptyLocusForFlippingPartsNode.hpp>
#include <tijcore/btnodes/DestroyPartAtSourceNode.hpp>
#include <tijcore/btnodes/FindClosesHintPoseForTargetNode.hpp>
#include <tijcore/btnodes/GetAuxiliarLocusRelativePoseNode.hpp>
#include <tijcore/btnodes/GetCurrentRobotPoseNode.hpp>
#include <tijcore/btnodes/GetDestinationLocusRelativePoseNode.hpp>
#include <tijcore/btnodes/GetDropBucketPoseNode.hpp>
#include <tijcore/btnodes/GetGripperIn3DPoseCartesianSpaceNode.hpp>
#include <tijcore/btnodes/GetGripperIn3DPoseJoinSpaceNode.hpp>
#include <tijcore/btnodes/GetRobotArmInRestingPoseNode.hpp>
#include <tijcore/btnodes/GetRobotGripperAttachementStateNode.hpp>
#include <tijcore/btnodes/GetRobotGripperOffNode.hpp>
#include <tijcore/btnodes/GetRobotGripperOnNode.hpp>
#include <tijcore/btnodes/GetRobotGripperToolTypeNode.hpp>
#include <tijcore/btnodes/GetRobotHealthStateNode.hpp>
#include <tijcore/btnodes/GetRobotInSafePoseNearTargetNode.hpp>
#include <tijcore/btnodes/GetRobotNameNode.hpp>
#include <tijcore/btnodes/GetRobotTo2DPoseNode.hpp>
#include <tijcore/btnodes/GetRobotToFlipPartAroundNode.hpp>
#include <tijcore/btnodes/GetSourceLocusRelativePoseNode.hpp>
#include <tijcore/btnodes/GetToolTablePoseNode.hpp>
#include <tijcore/btnodes/HackyDestinationPoseUpdateNode.hpp>
#include <tijcore/btnodes/HackyPartHeightCompensationNode.hpp>
#include <tijcore/btnodes/LockAccessToVolumeAtPoseNode.hpp>
#include <tijcore/btnodes/LockAccessToVolumeBetweenPosesNode.hpp>
#include <tijcore/btnodes/LockMovableTrayInAGVNode.hpp>
#include <tijcore/btnodes/LogErrorNode.hpp>
#include <tijcore/btnodes/LogInfoNode.hpp>
#include <tijcore/btnodes/LogWarningNode.hpp>
#include <tijcore/btnodes/ManeouverTypeIsNode.hpp>
#include <tijcore/btnodes/PartIsPumpThatRequiresFlippingNode.hpp>
#include <tijcore/btnodes/PosesAreWithinRangeNode.hpp>
#include <tijcore/btnodes/RandomizeTargetPoseNode.hpp>
#include <tijcore/btnodes/ReleaseAccessToLockedVolumeNode.hpp>
#include <tijcore/btnodes/RemoveRobotGripperPayloadEnvelopeNode.hpp>
#include <tijcore/btnodes/RotateRobotToFaceTargetNode.hpp>
#include <tijcore/btnodes/SetRobotGripperPayloadEnvelopeNode.hpp>
#include <tijcore/btnodes/SetRobotGripperStateNode.hpp>
#include <tijcore/btnodes/SetRobotGripperToolTypeNode.hpp>
#include <tijcore/btnodes/SwapDstAndAuxiliarLociNode.hpp>
#include <tijcore/btnodes/SwapSourceAndDestinationLociNode.hpp>
#include <tijcore/btnodes/SwapSrcAndAuxiliarLociNode.hpp>
#include <tijcore/btnodes/TestIfRobotReachesPoseNode.hpp>
#include <tijcore/btnodes/ToolTypesAreTheSameNode.hpp>
#include <tijcore/btnodes/TraceLoggerDecoratorNode.hpp>
#include <tijcore/btnodes/WeAreInASharedWorkspaceNode.hpp>

namespace tijcore
{
namespace
{
void factoryLoaderMethod(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<AbortCurrentActionNode>("AbortCurrentAction");
  factory.registerNodeType<CalculatePayloadIntoEndEffectorTransformNode>(
      "CalculatePayloadIntoEndEffectorTransform");
  factory.registerNodeType<CalculateVerticalDropPoseNode>("CalculateVerticalDropPose");
  factory.registerNodeType<CalculateVerticalGripEndEffectorPoseNode>(
      "CalculateVerticalGripEndEffectorPose");
  factory.registerNodeType<CalculateVerticalLandingPoseNode>("CalculateVerticalLandingPose");
  factory.registerNodeType<ContactPartFromAboveAndGraspNode>("ContactPartFromAboveAndGrasp");
  factory.registerNodeType<GetGripperIn3DPoseCartesianSpaceNode>(
      "GetGripperIn3DPoseCartesianSpace");
  factory.registerNodeType<GetGripperIn3DPoseJoinSpaceNode>("GetGripperIn3DPoseJoinSpace");
  factory.registerNodeType<GetRobotArmInRestingPoseNode>("GetRobotArmInRestingPose");
  factory.registerNodeType<GetRobotGripperAttachementStateNode>("GetRobotGripperAttachementState");
  factory.registerNodeType<GetRobotGripperOffNode>("GetRobotGripperOff");
  factory.registerNodeType<GetRobotGripperOnNode>("GetRobotGripperOn");
  factory.registerNodeType<GetRobotGripperToolTypeNode>("GetRobotGripperToolType");
  factory.registerNodeType<GetRobotHealthStateNode>("GetRobotHealthState");
  factory.registerNodeType<GetRobotInSafePoseNearTargetNode>("GetRobotInSafePoseNearTarget");
  factory.registerNodeType<GetRobotNameNode>("GetRobotName");
  factory.registerNodeType<GetRobotTo2DPoseNode>("GetRobotTo2DPose");
  factory.registerNodeType<RemoveRobotGripperPayloadEnvelopeNode>(
      "RemoveRobotGripperPayloadEnvelope");
  factory.registerNodeType<SetRobotGripperPayloadEnvelopeNode>("SetRobotGripperPayloadEnvelope");
  factory.registerNodeType<SetRobotGripperStateNode>("SetRobotGripperState");
  factory.registerNodeType<SetRobotGripperToolTypeNode>("SetRobotGripperToolType");
  factory.registerNodeType<TestIfRobotReachesPoseNode>("TestIfRobotReachesPose");
  factory.registerNodeType<GetSourceLocusRelativePoseNode>("GetSourceLocusRelativePose");
  factory.registerNodeType<GetDestinationLocusRelativePoseNode>("GetDestinationLocusRelativePose");
  factory.registerNodeType<SwapSourceAndDestinationLociNode>("SwapSourceAndDestinationLoci");
  factory.registerNodeType<DestroyPartAtSourceNode>("DestroyPartAtSource");
  factory.registerNodeType<GetDropBucketPoseNode>("GetDropBucketPose");
  factory.registerNodeType<ToolTypesAreTheSameNode>("ToolTypesAreTheSame");
  factory.registerNodeType<GetToolTablePoseNode>("GetToolTablePose");
  factory.registerNodeType<CalculateEnvelopeAndOffsetForVerticalPickUpNode>(
      "CalculateEnvelopeAndOffsetForVerticalPickUp");
  factory.registerNodeType<LogErrorNode>("LogError");
  factory.registerNodeType<LogWarningNode>("LogWarning");
  factory.registerNodeType<LogInfoNode>("LogInfo");
  factory.registerNodeType<TraceLoggerDecoratorNode>("TraceLoggerDecorator");
  factory.registerNodeType<RotateRobotToFaceTargetNode>("RotateRobotToFaceTarget");
  factory.registerNodeType<PosesAreWithinRangeNode>("PosesAreWithinRange");
  factory.registerNodeType<LockAccessToVolumeAtPoseNode>("LockAccessToVolumeAtPose");
  factory.registerNodeType<LockAccessToVolumeBetweenPosesNode>("LockAccessToVolumeBetweenPoses");
  factory.registerNodeType<ReleaseAccessToLockedVolumeNode>("ReleaseAccessToLockedVolume");
  factory.registerNodeType<RandomizeTargetPoseNode>("RandomizeTargetPose");
  factory.registerNodeType<GetCurrentRobotPoseNode>("GetCurrentRobotPose");
  factory.registerNodeType<FindClosesHintPoseForTargetNode>("FindClosesHintPoseForTarget");
  factory.registerNodeType<LockMovableTrayInAGVNode>("LockMovableTrayInAGV");
  factory.registerNodeType<CalculateEndEffectorPoseFromPayloadPoseNode>(
      "CalculateEndEffectorPoseFromPayloadPose");
  factory.registerNodeType<ManeouverTypeIsNode>("ManeouverTypeIs");
  factory.registerNodeType<WeAreInASharedWorkspaceNode>("WeAreInASharedWorkspace");
  factory.registerNodeType<CalculateRegulatorPreInsertAndInsertPosesNode>(
      "CalculateRegulatorPreInsertAndInsertPoses");
  factory.registerNodeType<CalculateSensorPreInsertAndInsertPosesNode>(
      "CalculateSensorPreInsertAndInsertPoses");
  factory.registerNodeType<PartIsPumpThatRequiresFlippingNode>("PartIsPumpThatRequiresFlipping");
  factory.registerNodeType<GetRobotToFlipPartAroundNode>("GetRobotToFlipPartAround");
  factory.registerNodeType<SwapSrcAndAuxiliarLociNode>("SwapSrcAndAuxiliarLoci");
  factory.registerNodeType<SwapDstAndAuxiliarLociNode>("SwapDstAndAuxiliarLoci");
  factory.registerNodeType<SwapDstAndAuxiliarLociNode>("GetAuxiliarLocusRelativePose");
  factory.registerNodeType<CreateEmptyLocusForFlippingPartsNode>(
      "CreateEmptyLocusForFlippingParts");
  factory.registerNodeType<HackyPartHeightCompensationNode>("HackyPartHeightCompensation");
  factory.registerNodeType<HackyDestinationPoseUpdateNode>("HackyDestinationPoseUpdate");
  factory.registerNodeType<CalibrateSourcePartFromPerceptionIfPossibleNode>(
      "CalibrateSourcePartFromPerceptionIfPossible");
  factory.registerNodeType<CalculatePayloadIntoEndEffectorTransformFromCamerasIfPossibleNode>(
      "CalculatePayloadIntoEndEffectorTransformFromCamerasIfPossible");
};

}  // namespace

BTRobotTaskFactory::BTRobotTaskFactory(const std::string& behavior_file_path,
                                       const ResourceManagerInterface::SharedPtr& resource_manager,
                                       const Toolbox::SharedPtr& toolbox)
  : behavior_file_path_{ behavior_file_path }
  , resource_manager_{ resource_manager }
  , toolbox_{ toolbox }
{
  behavior_tree_base_builder_ =
      std::make_unique<behavior_tree_extras::BehaviorTreeBuilder>(factoryLoaderMethod);
}

RobotTaskInterface::Ptr BTRobotTaskFactory::getRemoveBrokenPartTask(
    ResourceManagerInterface::ManagedLocusHandle&& source_locus,
    ResourceManagerInterface::PickAndPlaceRobotHandle&& robot) const
{
  auto bt_task_parameters = std::make_shared<BTTaskParameters>();
  bt_task_parameters->resource_manager = resource_manager_;
  bt_task_parameters->toolbox = toolbox_;
  bt_task_parameters->src_locus = std::move(source_locus);
  bt_task_parameters->primary_robot = std::move(robot);

  auto blackboard = BT::Blackboard::create();
  blackboard->set<BTTaskParameters::SharedPtr>(  // NOLINT(build/include_what_you_use)
      "task_parameters", std::move(bt_task_parameters));

  auto task_tree = behavior_tree_base_builder_->createTree()
                       .addFileDescription(behavior_file_path_)                      //
                       .addBlackboard(blackboard)                                    //
                       .addRootName("RemoveBrokenPartTaskRoot")                      //
                       .addLogger(std::make_unique<BehaviorTreeTransitionLogger>())  //
                       .build();
  return std::make_unique<BehaviorTreeWrappedTask>(std::move(task_tree));
}

RobotTaskInterface::Ptr BTRobotTaskFactory::getPickAndPlacePartTask(
    ResourceManagerInterface::ManagedLocusHandle&& src_locus,
    ResourceManagerInterface::ManagedLocusHandle&& dst_locus,
    ResourceManagerInterface::PickAndPlaceRobotHandle&& robot) const
{
  auto bt_task_parameters = std::make_shared<BTTaskParameters>();
  bt_task_parameters->resource_manager = resource_manager_;
  bt_task_parameters->toolbox = toolbox_;
  bt_task_parameters->src_locus = std::move(src_locus);
  bt_task_parameters->dst_locus = std::move(dst_locus);
  bt_task_parameters->primary_robot = std::move(robot);

  auto blackboard = BT::Blackboard::create();
  blackboard->set<BTTaskParameters::SharedPtr>(  // NOLINT(build/include_what_you_use)
      "task_parameters", std::move(bt_task_parameters));

  auto task_tree = behavior_tree_base_builder_->createTree()
                       .addFileDescription(behavior_file_path_)                      //
                       .addBlackboard(blackboard)                                    //
                       .addRootName("PickAndPlacePartRoot")                          //
                       .addLogger(std::make_unique<BehaviorTreeTransitionLogger>())  //
                       .build();
  return std::make_unique<BehaviorTreeWrappedTask>(std::move(task_tree));
}

RobotTaskInterface::Ptr BTRobotTaskFactory::getPickAndPlaceMovableTrayTask(
    ResourceManagerInterface::ManagedLocusHandle&& src_locus,
    ResourceManagerInterface::ManagedLocusHandle&& dst_locus,
    ResourceManagerInterface::PickAndPlaceRobotHandle&& robot, const tijcore::AgvId& agv_id) const
{
  auto bt_task_parameters = std::make_shared<BTTaskParameters>();
  bt_task_parameters->resource_manager = resource_manager_;
  bt_task_parameters->toolbox = toolbox_;
  bt_task_parameters->src_locus = std::move(src_locus);
  bt_task_parameters->dst_locus = std::move(dst_locus);
  bt_task_parameters->primary_robot = std::move(robot);
  bt_task_parameters->agv_id = agv_id;

  auto blackboard = BT::Blackboard::create();
  blackboard->set<BTTaskParameters::SharedPtr>(  // NOLINT(build/include_what_you_use)
      "task_parameters", std::move(bt_task_parameters));

  auto task_tree = behavior_tree_base_builder_->createTree()
                       .addFileDescription(behavior_file_path_)                      //
                       .addBlackboard(blackboard)                                    //
                       .addRootName("PickAndPlaceMovableTrayRoot")                   //
                       .addLogger(std::make_unique<BehaviorTreeTransitionLogger>())  //
                       .build();
  return std::make_unique<BehaviorTreeWrappedTask>(std::move(task_tree));
}

RobotTaskInterface::Ptr BTRobotTaskFactory::getSubmitKittingShipmentTask(
    const std::string& kitting_tray_name, const StationId& destination_station,
    const ShipmentType& shipment_type) const
{
  if (!agv::isValid(kitting_tray_name))
  {
    throw std::invalid_argument{ kitting_tray_name + " is not a valid agv getRobotName()!" };
  }
  return std::make_unique<SubmitKittingShipmentTask>(toolbox_, kitting_tray_name,
                                                     destination_station, shipment_type);
}

RobotTaskInterface::Ptr BTRobotTaskFactory::getSubmitAssemblyShipmentTask(
    const std::string& assembly_tray_name, const ShipmentType& shipment_type) const
{
  if (!station_id::isValid(assembly_tray_name))
  {
    throw std::invalid_argument{ assembly_tray_name + " is not a assembly station id!" };
  }
  return std::make_unique<SubmitAssemblyShipmentTask>(toolbox_, assembly_tray_name, shipment_type);
}

}  // namespace tijcore
