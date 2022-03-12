/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <string>

// roscpp
#include <geometry_msgs/Pose.h>

// tijcore
#include <tijcore/datatypes/AgvId.hpp>
#include <tijcore/datatypes/StationId.hpp>
#include <tijmath/Pose3.hpp>

namespace tijros
{
namespace utils
{
tijmath::Pose3 convertGeoPoseToCorePose(const geometry_msgs::Pose& geo_pose);

geometry_msgs::Pose convertCorePoseToGeoPose(const tijmath::Pose3& core_pose);

// TODO(glpuga) this conversion should not be hardcoded
std::string convertAgvIdToKitTrayFrameId(const tijcore::AgvId agv_id);

// TODO(glpuga) this conversion should not be hardcoded
std::string convertAgvIdToBriefcaseFrameId(const tijcore::StationId& station_id);

}  // namespace utils

}  // namespace tijros
