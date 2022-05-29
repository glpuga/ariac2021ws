/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <string>

// external
#include <nist_gear/Proximity.h>

// project
#include <tijcore/abstractions/HumanMonitorServiceInterface.hpp>
#include <tijcore/abstractions/SpatialMutualExclusionManagerInterface.hpp>
#include <tijmath/RelativePose3.hpp>

namespace tijros
{
class HumanMonitorService : public tijcore::HumanMonitorServiceInterface
{
public:
  HumanMonitorService(const ros::NodeHandle& nh, const std::string& human_monitor_id,
                      const std::string& table_sensor_topic, const std::string& pen_sensor_topic,
                      const tijmath::RelativePose3& table_pose,
                      const tijmath::RelativePose3& pen_pose, tijcore::Toolbox::SharedPtr toolbox);

private:
  ros::NodeHandle nh_;
  std::string human_monitor_id_;

  tijmath::RelativePose3 table_pose_;
  tijmath::RelativePose3 pen_pose_;

  ros::Subscriber table_break_beam_sub_;
  ros::Subscriber pen_break_beam_sub_;

  tijcore::SpatialMutualExclusionManagerInterface::SharedPtr mutual_exclusion_manager_;

  std::optional<tijcore::SpatialMutualExclusionManagerInterface::VolumeHandle>
      space_exclusion_handle_opt_;

  void tableBreakbeamCallback(nist_gear::Proximity::ConstPtr msg);

  void penBreakbeamCallback(nist_gear::Proximity::ConstPtr msg);
};

}  // namespace tijros
