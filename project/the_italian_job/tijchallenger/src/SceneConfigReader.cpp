/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// standard library
#include <string>
#include <vector>

// project
#include <tijchallenger/SceneConfigReader.hpp>
#include <tijmath/math_utilities.hpp>

namespace tijchallenger
{
namespace
{
constexpr char world_frame_id_[] = "world";

}  // namespace

using tijmath::utils::angles::degreesToRadians;

const std::string& SceneConfigReader::getWorldFrameId() const
{
  static const std::string data{ world_frame_id_ };
  return data;
};

const tijmath::RelativePose3& SceneConfigReader::getDropBucketPose() const
{
  static const tijmath::RelativePose3 data{
    world_frame_id_, tijmath::Pose3{ tijmath::Position::fromVector(-2.2, 0.0, 1.0), {} }
  };
  return data;
}

const tijmath::RelativePose3& SceneConfigReader::getGripperToolSwappingTablePose() const
{
  static const tijmath::RelativePose3 data{
    world_frame_id_, tijmath::Pose3{ tijmath::Position::fromVector(-3.71, 6.26, 1.5), {} }
  };
  return data;
}

const std::vector<SceneConfigReader::BinData>& SceneConfigReader::getListOfBins() const
{
  static const std::vector<BinData> data{
    {
        "bin1",
        "bin1_frame",
        "agv12_near_kitting_access_space",
    },
    {
        "bin2",
        "bin2_frame",
        "agv12_near_kitting_access_space",
    },
    {
        "bin3",
        "bin3_frame",
        "agv12_far_kitting_access_space",
    },
    {
        "bin4",
        "bin4_frame",
        "agv12_far_kitting_access_space",
    },
    {
        "bin5",
        "bin5_frame",
        "agv34_near_kitting_access_space",
    },
    {
        "bin6",
        "bin6_frame",
        "agv34_near_kitting_access_space",
    },
    {
        "bin7",
        "bin7_frame",
        "agv34_far_kitting_access_space",
    },
    {
        "bin8",
        "bin8_frame",
        "agv34_far_kitting_access_space",
    },
  };
  return data;
}

const std::vector<SceneConfigReader::DeviceData>& SceneConfigReader::getListOfLogicalCameras() const
{
  static const std::vector<SceneConfigReader::DeviceData> data{
    { "logical_camera_bin_agv1", "logical_camera_bin_agv1_frame" },
    { "logical_camera_bin_agv2", "logical_camera_bin_agv2_frame" },
    { "logical_camera_bin_agv3", "logical_camera_bin_agv3_frame" },
    { "logical_camera_bin_agv4", "logical_camera_bin_agv4_frame" },
    { "logical_camera_as1", "logical_camera_as1_frame" },
    { "logical_camera_as2", "logical_camera_as2_frame" },
    { "logical_camera_as3", "logical_camera_as3_frame" },
    { "logical_camera_as4", "logical_camera_as4_frame" },
    { "logical_camera_agv1_as1", "logical_camera_agv1_as1_frame" },
    { "logical_camera_agv1_as2", "logical_camera_agv1_as2_frame" },
    { "logical_camera_agv2_as1", "logical_camera_agv2_as1_frame" },
    { "logical_camera_agv2_as2", "logical_camera_agv2_as2_frame" },
    { "logical_camera_agv3_as3", "logical_camera_agv3_as3_frame" },
    { "logical_camera_agv3_as4", "logical_camera_agv3_as4_frame" },
    { "logical_camera_agv4_as3", "logical_camera_agv4_as3_frame" },
    { "logical_camera_agv4_as4", "logical_camera_agv4_as4_frame" },
    { "logical_camera_conveyor_belt_1", "logical_camera_conveyor_belt_1_frame" },
    { "logical_camera_conveyor_belt_2", "logical_camera_conveyor_belt_2_frame" },
  };
  return data;
}

const std::vector<SceneConfigReader::DeviceData>&
SceneConfigReader::getListOfQualityControlSensors() const
{
  static const std::vector<SceneConfigReader::DeviceData> data{
    { "quality_control_sensor_1", "quality_control_sensor_1_frame" },
    { "quality_control_sensor_2", "quality_control_sensor_2_frame" },
    { "quality_control_sensor_3", "quality_control_sensor_3_frame" },
    { "quality_control_sensor_4", "quality_control_sensor_4_frame" },
  };
  return data;
}

const std::vector<SceneConfigReader::DeviceData>& SceneConfigReader::getListOfAgvs() const
{
  static const std::vector<SceneConfigReader::DeviceData> data{
    { "agv1", "kit_tray_1", "agv12_near_kitting_access_space" },
    { "agv2", "kit_tray_2", "agv12_near_kitting_access_space" },
    { "agv3", "kit_tray_3", "agv34_near_kitting_access_space" },
    { "agv4", "kit_tray_4", "agv34_near_kitting_access_space" },
  };
  return data;
}

const std::vector<SceneConfigReader::DeviceData>&
SceneConfigReader::getListOfAssemblyStations() const
{
  static const std::vector<DeviceData> data{
    { "as1", "briefcase_1", "as1_access_space" },
    { "as2", "briefcase_2", "as2_access_space" },
    { "as3", "briefcase_3", "as3_access_space" },
    { "as4", "briefcase_4", "as4_access_space" },
  };
  return data;
}

const std::vector<SceneConfigReader::ConveyorBeltData>&
SceneConfigReader::getListOfConveyorBelts() const
{
  static const std::vector<ConveyorBeltData> data{
    { "conveyor_belt", "corrected_belt_frame", "belt_frame_surface", "conveyor_belt_access_space",
      0.2 },
  };
  return data;
}

const std::vector<tijmath::RelativePose3>& SceneConfigReader::getListOfGantryPlanningHints() const
{
  static const std::vector<tijmath::RelativePose3> data{
    { world_frame_id_, tijmath::Position::fromVector(-3.5, 3.1, 0),
      tijmath::Rotation::fromRollPitchYaw(0.0, 0.0, degreesToRadians(-45)) },
    { world_frame_id_, tijmath::Position::fromVector(-3.5, 2.9, 0),
      tijmath::Rotation::fromRollPitchYaw(0.0, 0.0, degreesToRadians(-125)) },
    // ---
    { world_frame_id_, tijmath::Position::fromVector(-3.5, -2.9, 0),
      tijmath::Rotation::fromRollPitchYaw(0.0, 0.0, degreesToRadians(-45)) },
    { world_frame_id_, tijmath::Position::fromVector(-3.5, -3.0, 0),
      tijmath::Rotation::fromRollPitchYaw(0.0, 0.0, degreesToRadians(-125)) },
    // ---
    { world_frame_id_, tijmath::Position::fromVector(-5, 3.0, 0),
      tijmath::Rotation::fromRollPitchYaw(0.0, 0.0, degreesToRadians(90)) },
    { world_frame_id_, tijmath::Position::fromVector(-5, -3.0, 0),
      tijmath::Rotation::fromRollPitchYaw(0.0, 0.0, degreesToRadians(90)) },
    { world_frame_id_, tijmath::Position::fromVector(-10, 3.0, 0),
      tijmath::Rotation::fromRollPitchYaw(0.0, 0.0, degreesToRadians(90)) },
    { world_frame_id_, tijmath::Position::fromVector(-10, -3.0, 0),
      tijmath::Rotation::fromRollPitchYaw(0.0, 0.0, degreesToRadians(90)) },
  };
  return data;
}

const std::vector<tijmath::RelativePose3>& SceneConfigReader::getListOfSafeWaitingSpotHints() const
{
  static const std::vector<tijmath::RelativePose3> data{
    { world_frame_id_, tijmath::Position::fromVector(-4.0, 3.1, 0),
      tijmath::Rotation::fromRollPitchYaw(0.0, 0.0, degreesToRadians(-45)) },
    { world_frame_id_, tijmath::Position::fromVector(-4.0, 2.9, 0),
      tijmath::Rotation::fromRollPitchYaw(0.0, 0.0, degreesToRadians(-125)) },
    { world_frame_id_, tijmath::Position::fromVector(-4.0, -2.9, 0),
      tijmath::Rotation::fromRollPitchYaw(0.0, 0.0, degreesToRadians(-45)) },
    { world_frame_id_, tijmath::Position::fromVector(-4.0, -3.1, 0),
      tijmath::Rotation::fromRollPitchYaw(0.0, 0.0, degreesToRadians(-125)) },
    // ---
    { world_frame_id_, tijmath::Position::fromVector(-5, 3.0, 0),
      tijmath::Rotation::fromRollPitchYaw(0.0, 0.0, degreesToRadians(90)) },
    { world_frame_id_, tijmath::Position::fromVector(-5, -3.0, 0),
      tijmath::Rotation::fromRollPitchYaw(0.0, 0.0, degreesToRadians(90)) },
    { world_frame_id_, tijmath::Position::fromVector(-10, 3.0, 0),
      tijmath::Rotation::fromRollPitchYaw(0.0, 0.0, degreesToRadians(90)) },
    { world_frame_id_, tijmath::Position::fromVector(-10, -3.0, 0),
      tijmath::Rotation::fromRollPitchYaw(0.0, 0.0, degreesToRadians(90)) },
  };
  return data;
}

}  // namespace tijchallenger
