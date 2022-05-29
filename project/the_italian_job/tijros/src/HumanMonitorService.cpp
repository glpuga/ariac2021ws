/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <string>
#include <utility>

// ros
#include <ros/ros.h>

// tijcore
#include <tijcore/coremodels/Toolbox.hpp>
#include <tijlogger/logger.hpp>
#include <tijros/HumanMonitorService.hpp>

namespace tijros
{
namespace
{
constexpr int default_queue_len_ = 10;
const double safety_radius_ = 2.0;
}  // namespace

HumanMonitorService::HumanMonitorService(const ros::NodeHandle& nh,
                                         const std::string& human_monitor_id,
                                         const std::string& table_sensor_topic,
                                         const std::string& pen_sensor_topic,
                                         const tijmath::RelativePose3& table_pose,
                                         const tijmath::RelativePose3& pen_pose,
                                         tijcore::Toolbox::SharedPtr toolbox)
  : nh_{ nh }
  , human_monitor_id_{ human_monitor_id }
  , table_pose_{ table_pose }
  , pen_pose_{ pen_pose }
  , mutual_exclusion_manager_{ toolbox->getSpatialMutualExclusionManager() }

{
  table_break_beam_sub_ = nh_.subscribe(table_sensor_topic, default_queue_len_,
                                        &HumanMonitorService::tableBreakbeamCallback, this);

  pen_break_beam_sub_ = nh_.subscribe(pen_sensor_topic, default_queue_len_,
                                      &HumanMonitorService::penBreakbeamCallback, this);
}

void HumanMonitorService::tableBreakbeamCallback(nist_gear::Proximity::ConstPtr msg)
{
  if (msg->object_detected && !space_exclusion_handle_opt_)
  {
    ERROR("Table break beam detected, locking access to the human track at {}", human_monitor_id_);
    space_exclusion_handle_opt_ =
        mutual_exclusion_manager_->lockSpheresPathVolume(table_pose_, pen_pose_, safety_radius_);
  }
}

void HumanMonitorService::penBreakbeamCallback(nist_gear::Proximity::ConstPtr msg)
{
  if (msg->object_detected && space_exclusion_handle_opt_)
  {
    ERROR("Pen break beam detected, unlocking access to the human track at {}", human_monitor_id_);
    space_exclusion_handle_opt_.reset();
  }
}

}  // namespace tijros
