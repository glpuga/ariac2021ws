/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// standard library
#include <stdexcept>
#include <string>
#include <vector>

// tijcore
#include <tijros/utils/utils.hpp>

namespace tijros {

namespace utils {

tijcore::Pose3 convertGeoPoseToCorePose(const geometry_msgs::Pose &geo_pose) {
  auto output_position = tijcore::Position::fromVector(
      geo_pose.position.x, geo_pose.position.y, geo_pose.position.z);

  auto output_rotation = tijcore::Rotation::fromQuaternion(
      geo_pose.orientation.x, geo_pose.orientation.y, geo_pose.orientation.z,
      geo_pose.orientation.w);

  tijcore::Pose3 output_pose{output_position, output_rotation};

  return output_pose;
}

geometry_msgs::Pose convertCorePoseToGeoPose(const tijcore::Pose3 &core_pose) {
  geometry_msgs::Pose geo_pose;

  auto position_vector = core_pose.position().vector();
  geo_pose.position.x = position_vector.x();
  geo_pose.position.y = position_vector.y();
  geo_pose.position.z = position_vector.z();

  auto rotation_quaternion = core_pose.rotation().quaternion();
  geo_pose.orientation.x = rotation_quaternion.x();
  geo_pose.orientation.y = rotation_quaternion.y();
  geo_pose.orientation.z = rotation_quaternion.z();
  geo_pose.orientation.w = rotation_quaternion.w();

  return geo_pose;
}

std::string convertAgvIdToKitTrayFrameId(const tijcore::AgvId agv_id) {
  std::string kit_tray_frame_id;
  switch (agv_id) {
  case tijcore::AgvId::agv1:
    kit_tray_frame_id = "kit_tray_1";
    break;
  case tijcore::AgvId::agv2:
    kit_tray_frame_id = "kit_tray_2";
    break;
  case tijcore::AgvId::agv3:
    kit_tray_frame_id = "kit_tray_3";
    break;
  case tijcore::AgvId::agv4:
    kit_tray_frame_id = "kit_tray_4";
    break;
  case tijcore::AgvId::any:
    throw std::invalid_argument{"agv_id valued any cannot be given a frame id"};
    break;
  }
  return kit_tray_frame_id;
}

std::string
convertAgvIdToBriefcaseFrameId(const tijcore::StationId &station_id) {
  std::string briefcase_frame_id;
  switch (station_id) {
  case tijcore::StationId::as1:
    briefcase_frame_id = "briefcase_1";
    break;
  case tijcore::StationId::as2:
    briefcase_frame_id = "briefcase_2";
    break;
  case tijcore::StationId::as3:
    briefcase_frame_id = "briefcase_3";
    break;
  case tijcore::StationId::as4:
    briefcase_frame_id = "briefcase_4";
    break;
  case tijcore::StationId::ks1:
  case tijcore::StationId::ks2:
  case tijcore::StationId::ks3:
  case tijcore::StationId::ks4:
    throw std::invalid_argument{
        "a target station is in a kitting station is not valid"};
    break;
  case tijcore::StationId::any:
    throw std::invalid_argument{
        "station_id valued any cannot be given a frame id"};
    break;
  }
  return briefcase_frame_id;
}

} // namespace utils

} // namespace tijros
