/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

// roscpp
#include <nist_gear/Orders.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

// tijcore
#include <tijcore/abstractions/ProcessManagementInterface.hpp>

namespace tijros
{
class ROSProcessManagement : public tijcore::ProcessManagementInterface
{
public:
  explicit ROSProcessManagement(const ros::NodeHandle& nh);

  ErrorWithReason startCompetition() const override;

  ErrorWithReason endCompetition() const override;

  std::string getCompetitionState() const override;

  std::vector<tijcore::Order> getOrders() override;

  ErrorWithReason submitAgvToAssemblyStation(const tijcore::AgvId& agv_id,
                                             const tijcore::StationId& destination_station,
                                             const std::string& shipment_type) const override;

  std::string getAgvState(const tijcore::AgvId& agv_id) const override;

  ErrorWithReason submitAssemblyStation(const tijcore::StationId& station,
                                        const std::string& shipment_type) const override;

  tijcore::StationId getAgvStation(const tijcore::AgvId& agv_id) const override;

private:
  mutable std::mutex mutex_;

  ros::NodeHandle nh_;

  ros::Subscriber competition_state_sub_;
  ros::Subscriber orders_sub_;

  ros::Subscriber agv1_state_sub_;
  ros::Subscriber agv2_state_sub_;
  ros::Subscriber agv3_state_sub_;
  ros::Subscriber agv4_state_sub_;

  ros::Subscriber agv1_station_sub_;
  ros::Subscriber agv2_station_sub_;
  ros::Subscriber agv3_station_sub_;
  ros::Subscriber agv4_station_sub_;

  std::string latest_competition_state_data_;

  std::map<tijcore::AgvId, std::string> agv_state_data_;
  std::map<tijcore::AgvId, tijcore::StationId> agv_station_data_;

  mutable std::vector<tijcore::Order> orders_backlog_;

  void competitionStateCallback(std_msgs::String::ConstPtr msg);

  void ordersCallback(nist_gear::Orders::ConstPtr msg);

  void agv1StateCallback(std_msgs::String::ConstPtr msg);

  void agv2StateCallback(std_msgs::String::ConstPtr msg);

  void agv3StateCallback(std_msgs::String::ConstPtr msg);

  void agv4StateCallback(std_msgs::String::ConstPtr msg);

  void agv1StationCallback(std_msgs::String::ConstPtr msg);

  void agv2StationCallback(std_msgs::String::ConstPtr msg);

  void agv3StationCallback(std_msgs::String::ConstPtr msg);

  void agv4StationCallback(std_msgs::String::ConstPtr msg);
};

}  // namespace tijros
