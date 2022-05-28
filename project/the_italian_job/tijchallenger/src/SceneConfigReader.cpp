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
    world_frame_id_, tijmath::Pose3{ tijmath::Position::fromVector(-3.71, 6.46, 1.5), {} }
  };
  return data;
}

const std::vector<SceneConfigReader::BinData>& SceneConfigReader::getListOfBins() const
{
  static const std::vector<BinData> data{
    {
        "bin1",
        "bin1_frame",
    },
    {
        "bin2",
        "bin2_frame",
    },
    {
        "bin3",
        "bin3_frame",
    },
    {
        "bin4",
        "bin4_frame",
    },
    {
        "bin5",
        "bin5_frame",
    },
    {
        "bin6",
        "bin6_frame",
    },
    {
        "bin7",
        "bin7_frame",
    },
    {
        "bin8",
        "bin8_frame",
    },
  };
  return data;
}

const std::vector<SceneConfigReader::TableData>& SceneConfigReader::getListOfTables() const
{
  static const std::vector<TableData> data{
    {
        "tray_table1",
        "tray_table1_frame",
    },
    {
        "tray_table2",
        "tray_table2_frame",
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
    { "logical_camera_bin_trays", "logical_camera_bin_trays_frame" },
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
    { "agv1", "kit_tray_1" },
    { "agv2", "kit_tray_2" },
    { "agv3", "kit_tray_3" },
    { "agv4", "kit_tray_4" },
  };
  return data;
}

const std::vector<SceneConfigReader::DeviceData>&
SceneConfigReader::getListOfAssemblyStations() const
{
  static const std::vector<DeviceData> data{
    { "as1", "briefcase_1" },
    { "as2", "briefcase_2" },
    { "as3", "briefcase_3" },
    { "as4", "briefcase_4" },
  };
  return data;
}

const std::vector<SceneConfigReader::ConveyorBeltData>&
SceneConfigReader::getListOfConveyorBelts() const
{
  static const std::vector<ConveyorBeltData> data{
    { "conveyor_belt", "corrected_belt_frame", "belt_frame_surface", 0.2 },
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
    { world_frame_id_, tijmath::Position::fromVector(-6.06, 6.16, 0),
      tijmath::Rotation::fromRollPitchYaw(0.0, 0.0, degreesToRadians(90)) },
    // ---
    { world_frame_id_, tijmath::Position::fromVector(-3.2, 0.0, 1.0),
      tijmath::Rotation::fromRollPitchYaw(0.0, 0.0, degreesToRadians(-90)) },
  };
  return data;
}

const std::vector<SceneConfigReader::PickAndPlacePoseHintsData>&
SceneConfigReader::getListOfApproachHints(const std::string& robot_name) const
{
  auto build_hint = [](const double x, const double y, const double deg) -> tijmath::RelativePose3 {
    return { world_frame_id_, tijmath::Position::fromVector(x, y, 0),
             tijmath::Rotation::fromRollPitchYaw(0.0, 0.0, degreesToRadians(deg)) };
  };

  static std::vector<PickAndPlacePoseHintsData> kitting_poses{
    { build_hint(-1.00, +4.50, degreesToRadians(0)),
      build_hint(-1.00, +4.00, degreesToRadians(0)) },
  };

  for (double y = -4.0; y < 4.0; y += 0.10)
  {
    SceneConfigReader::PickAndPlacePoseHintsData new_item;
    new_item.target_pose = build_hint(-1.00, y - 1.0, degreesToRadians(0));
    new_item.approach_pose = build_hint(-1.00, y, degreesToRadians(0));
    kitting_poses.push_back(new_item);
  }

  static const std::vector<PickAndPlacePoseHintsData> gantry_poses{
    //
    // left
    //
    // bins left
    { build_hint(-2.00, +3.00, degreesToRadians(0)),
      build_hint(-3.50, +3.00, degreesToRadians(0)) },
    // agvs left
    { build_hint(-2.26, +4.67, degreesToRadians(0)),
      build_hint(-2.20, +3.50, degreesToRadians(0)) },
    { build_hint(-2.26, +1.36, degreesToRadians(0)),
      build_hint(-2.20, +2.50, degreesToRadians(0)) },
    // first station left
    { build_hint(-5.50, +4.50, degreesToRadians(0)),
      build_hint(-5.00, +4.00, degreesToRadians(0)) },
    { build_hint(-5.50, +1.40, degreesToRadians(0)),
      build_hint(-5.00, +1.90, degreesToRadians(0)) },
    { build_hint(-7.50, +3.00, degreesToRadians(0)),
      build_hint(-5.50, +3.00, degreesToRadians(0)) },
    // second station left
    { build_hint(-10.50, +4.50, degreesToRadians(0)),
      build_hint(-10.00, +4.00, degreesToRadians(0)) },
    { build_hint(-10.50, +1.40, degreesToRadians(0)),
      build_hint(-10.00, +1.90, degreesToRadians(0)) },
    { build_hint(-12.50, +3.00, degreesToRadians(0)),
      build_hint(-10.50, +3.00, degreesToRadians(0)) },
    //
    // right
    //
    // bins right
    { build_hint(-2.00, -3.00, degreesToRadians(0)),
      build_hint(-3.50, -3.00, degreesToRadians(0)) },
    // agvs right
    { build_hint(-2.26, -4.67, degreesToRadians(0)),
      build_hint(-2.20, -3.50, degreesToRadians(0)) },
    { build_hint(-2.26, -1.36, degreesToRadians(0)),
      build_hint(-2.20, -2.50, degreesToRadians(0)) },
    // first station right
    { build_hint(-5.50, -4.50, degreesToRadians(0)),
      build_hint(-5.00, -4.00, degreesToRadians(0)) },
    { build_hint(-5.50, -1.40, degreesToRadians(0)),
      build_hint(-5.00, -1.90, degreesToRadians(0)) },
    { build_hint(-7.50, -3.00, degreesToRadians(0)),
      build_hint(-5.50, -3.00, degreesToRadians(0)) },
    // second station right
    { build_hint(-10.50, -4.50, degreesToRadians(0)),
      build_hint(-10.00, -4.00, degreesToRadians(0)) },
    { build_hint(-10.50, -1.40, degreesToRadians(0)),
      build_hint(-10.00, -1.90, degreesToRadians(0)) },
    { build_hint(-12.50, -3.00, degreesToRadians(0)),
      build_hint(-10.50, -3.00, degreesToRadians(0)) },
    //
    // drop bucket
    //
    { build_hint(-2.20, 0.00, degreesToRadians(0)), build_hint(-3.50, 0.00, degreesToRadians(0)) },
    //
    // trays tables
    //
    { build_hint(-6.56, +6.26, degreesToRadians(0)),
      build_hint(-5.61, +6.26, degreesToRadians(0)) },
    { build_hint(-5.61, +6.26, degreesToRadians(0)),
      build_hint(-6.56, +6.26, degreesToRadians(0)) },
  };

  if (robot_name == "kitting")
  {
    return kitting_poses;
  }
  else if (robot_name == "gantry")
  {
    return gantry_poses;
  }
  else
  {
    throw std::runtime_error("Unknown robot name: " + robot_name);
  }
}

}  // namespace tijchallenger
