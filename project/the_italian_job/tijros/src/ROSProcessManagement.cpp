/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// standard library
#include <mutex>
#include <string>
#include <utility>
#include <vector>

// roscpp
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

// competition
#include <nist_gear/AssemblyStationSubmitShipment.h>
#include <nist_gear/Order.h>
#include <nist_gear/SubmitKittingShipment.h>

// tijcore
#include <tijlogger/logger.hpp>
#include <tijros/ROSProcessManagement.hpp>
#include <tijros/utils/utils.hpp>

namespace tijros
{
namespace
{
constexpr int default_queue_len = 10;

constexpr char start_competition_service[] = "/ariac/start_competition";
constexpr char end_competition_service[] = "/ariac/end_competition";

constexpr char competition_state_topic[] = "/ariac/competition_state";

constexpr char orders_topic[] = "/ariac/orders";

constexpr char agv1_submit_shipment_service[] = "/ariac/agv1/submit_shipment";
constexpr char agv2_submit_shipment_service[] = "/ariac/agv2/submit_shipment";
constexpr char agv3_submit_shipment_service[] = "/ariac/agv3/submit_shipment";
constexpr char agv4_submit_shipment_service[] = "/ariac/agv4/submit_shipment";

constexpr char agv1_state_topic[] = "/ariac/agv1/state";
constexpr char agv2_state_topic[] = "/ariac/agv2/state";
constexpr char agv3_state_topic[] = "/ariac/agv3/state";
constexpr char agv4_state_topic[] = "/ariac/agv4/state";

constexpr char as1_submit_shipment_service[] = "/ariac/as1/submit_shipment";
constexpr char as2_submit_shipment_service[] = "/ariac/as2/submit_shipment";
constexpr char as3_submit_shipment_service[] = "/ariac/as3/submit_shipment";
constexpr char as4_submit_shipment_service[] = "/ariac/as4/submit_shipment";

constexpr char agv1_station_topic[] = "/ariac/agv1/station";
constexpr char agv2_station_topic[] = "/ariac/agv2/station";
constexpr char agv3_station_topic[] = "/ariac/agv3/station";
constexpr char agv4_station_topic[] = "/ariac/agv4/station";

const ros::Duration connection_timeout_{ 60 };

bool validateStationId(const std::string& station_id_str)
{
  if (!tijcore::station_id::isValid(station_id_str))
  {
    ERROR("Invalid station_id value received from topic: {}", station_id_str);
    return false;
  }
  auto station = tijcore::station_id::fromString(station_id_str);
  if (tijcore::station_id::isAny(station))
  {
    ERROR("station_id valued Any received from topic: {}", station_id_str);
    return false;
  }
  return true;
}

};  // namespace

using tijcore::AgvId;
using tijcore::ProcessManagementInterface;
using tijcore::StationId;

ROSProcessManagement::ROSProcessManagement(const ros::NodeHandle& nh) : nh_{ nh }
{
  competition_state_sub_ = nh_.subscribe(competition_state_topic, default_queue_len,
                                         &ROSProcessManagement::competitionStateCallback, this);

  orders_sub_ =
      nh_.subscribe(orders_topic, default_queue_len, &ROSProcessManagement::ordersCallback, this);

  agv1_state_sub_ = nh_.subscribe(agv1_state_topic, default_queue_len,
                                  &ROSProcessManagement::agv1StateCallback, this);
  agv2_state_sub_ = nh_.subscribe(agv2_state_topic, default_queue_len,
                                  &ROSProcessManagement::agv2StateCallback, this);
  agv3_state_sub_ = nh_.subscribe(agv3_state_topic, default_queue_len,
                                  &ROSProcessManagement::agv3StateCallback, this);
  agv4_state_sub_ = nh_.subscribe(agv4_state_topic, default_queue_len,
                                  &ROSProcessManagement::agv4StateCallback, this);

  agv1_station_sub_ = nh_.subscribe(agv1_station_topic, default_queue_len,
                                    &ROSProcessManagement::agv1StationCallback, this);
  agv2_station_sub_ = nh_.subscribe(agv2_station_topic, default_queue_len,
                                    &ROSProcessManagement::agv2StationCallback, this);
  agv3_station_sub_ = nh_.subscribe(agv3_station_topic, default_queue_len,
                                    &ROSProcessManagement::agv3StationCallback, this);
  agv4_station_sub_ = nh_.subscribe(agv4_station_topic, default_queue_len,
                                    &ROSProcessManagement::agv4StationCallback, this);
}

ProcessManagementInterface::ErrorWithReason ROSProcessManagement::startCompetition() const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  std_srvs::Trigger msg;
  ros::service::waitForService(start_competition_service, connection_timeout_);
  ros::service::call(start_competition_service, msg);
  if (!msg.response.success)
  {
    ERROR("Failed to start competition mode: {}", msg.response.message);
  }
  else
  {
    INFO("Started competition mode (msg: {})", msg.response.message);
  }
  return std::make_pair(msg.response.success, msg.response.message);
}

ProcessManagementInterface::ErrorWithReason ROSProcessManagement::endCompetition() const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  std_srvs::Trigger msg;
  ros::service::waitForService(end_competition_service, connection_timeout_);
  ros::service::call(end_competition_service, msg);
  if (!msg.response.success)
  {
    ERROR("Failed to end competition mode: {}", msg.response.message);
  }
  else
  {
    INFO("Ended competition mode (msg: {})", msg.response.message);
  }
  return std::make_pair(msg.response.success, msg.response.message);
}

std::string ROSProcessManagement::getCompetitionState() const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  return latest_competition_state_data_;
}

std::vector<tijcore::Order> ROSProcessManagement::getOrders()
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  auto orders_data_copy = orders_backlog_;
  orders_backlog_.clear();
  return orders_data_copy;
}

ProcessManagementInterface::ErrorWithReason ROSProcessManagement::submitAgvToAssemblyStation(
    const AgvId& agv_id, const tijcore::StationId& destination_station,
    const std::string& shipment_type) const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  nist_gear::SubmitKittingShipment msg;
  msg.request.assembly_station_name = tijcore::station_id::toString(destination_station);
  msg.request.shipment_type = shipment_type;

  {
    std::string service_id;
    switch (agv_id)
    {
      case AgvId::agv1:
        service_id = agv1_submit_shipment_service;
        break;
      case AgvId::agv2:
        service_id = agv2_submit_shipment_service;
        break;
      case AgvId::agv3:
        service_id = agv3_submit_shipment_service;
        break;
      case AgvId::agv4:
        service_id = agv4_submit_shipment_service;
        break;
      case AgvId::any:
        throw std::invalid_argument{ "Can't submit a shipment to from \"any\"" };
        break;
    }

    ros::service::waitForService(service_id, connection_timeout_);
    ros::service::call(service_id, msg);
  }

  if (!msg.response.success)
  {
    ERROR("Failed to submit shipment {}: {}", shipment_type, msg.response.message);
  }
  else
  {
    INFO("Submitted shipment {} (msg: {})", shipment_type, msg.response.message);
  }
  return std::make_pair(msg.response.success, msg.response.message);
}

std::string ROSProcessManagement::getAgvState(const AgvId& agv_id) const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  if (agv_state_data_.count(agv_id))
  {
    return agv_state_data_.at(agv_id);
  }
  ERROR("Requested state data for an agv that has not received any {}", agv_id);
  return "";
}

ProcessManagementInterface::ErrorWithReason ROSProcessManagement::submitAssemblyStation(
    const StationId& station_id, const std::string& shipment_type) const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  nist_gear::AssemblyStationSubmitShipment msg;
  msg.request.shipment_type = shipment_type;

  {
    std::string service_id;
    switch (station_id)
    {
      case StationId::as1:
        service_id = as1_submit_shipment_service;
        break;
      case StationId::as2:
        service_id = as2_submit_shipment_service;
        break;
      case StationId::as3:
        service_id = as3_submit_shipment_service;
        break;
      case StationId::as4:
        service_id = as4_submit_shipment_service;
        break;
      case StationId::ks1:
      case StationId::ks2:
      case StationId::ks3:
      case StationId::ks4:
        throw std::invalid_argument{ "Can't submit a shipment to from " +
                                     tijcore::station_id::toString(station_id) };
        break;
      case StationId::any:
        throw std::invalid_argument{ "Can't submit a shipment to from \"any\"" };
        break;
    }

    ros::service::waitForService(service_id, connection_timeout_);
    ros::service::call(service_id, msg);
  }

  auto message = "inspection_result = " + std::to_string(msg.response.inspection_result);

  if (!msg.response.success)
  {
    ERROR("Failed to submit shipment {} (message: {})", shipment_type, message);
  }
  else
  {
    INFO("Submitted shipment {} (message: {})", shipment_type, message);
  }
  return std::make_pair(msg.response.success, message);
}

StationId ROSProcessManagement::getAgvStation(const AgvId& agv_id) const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  if (agv_station_data_.count(agv_id))
  {
    return agv_station_data_.at(agv_id);
  }
  // TODO(glpuga) enable back when the topic is actually published!
  // ERROR("Requested station_id data for an agv that has not received any {}",
  //    agv_id);
  return StationId::ks1;
}

void ROSProcessManagement::competitionStateCallback(std_msgs::String::ConstPtr msg)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  latest_competition_state_data_ = msg->data;
}

void ROSProcessManagement::ordersCallback(nist_gear::Order::ConstPtr msg)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  tijcore::Order new_order;
  new_order.order_id = tijcore::OrderId(msg->order_id);

  auto convert_ros_product_to_core_product = [](const auto& ros_msg_product_description,
                                                const std::string& tray_frame_id) {
    return tijcore::ProductRequest{
      tijcore::PartId(ros_msg_product_description.type),
      tijmath::RelativePose3{ tray_frame_id,
                              utils::convertGeoPoseToCorePose(ros_msg_product_description.pose) }
    };
  };

  for (const auto& input_kitting_shipment : msg->kitting_shipments)
  {
    tijcore::KittingShipment output_kitting_shipment;
    output_kitting_shipment.shipment_type = input_kitting_shipment.shipment_type;
    output_kitting_shipment.station_id =
        tijcore::station_id::fromString(input_kitting_shipment.assembly_station);
    output_kitting_shipment.agv_id =
        tijcore::agv::fromString(input_kitting_shipment.tray_content.kit_tray);

    auto kit_tray_frame_id = utils::convertAgvIdToKitTrayFrameId(output_kitting_shipment.agv_id);

    for (const auto& input_product : input_kitting_shipment.tray_content.products)
    {
      output_kitting_shipment.products.push_back(
          convert_ros_product_to_core_product(input_product, kit_tray_frame_id));
    }
    new_order.kitting_shipments.push_back(output_kitting_shipment);
  }

  for (const auto& input_assembly_shipment : msg->assembly_shipments)
  {
    tijcore::AssemblyShipment output_assembly_shipment;
    output_assembly_shipment.station_id =
        tijcore::station_id::fromString(input_assembly_shipment.station_id);
    output_assembly_shipment.shipment_type = input_assembly_shipment.shipment_type;

    auto briefcase_frame_id =
        utils::convertAgvIdToBriefcaseFrameId(output_assembly_shipment.station_id);

    for (const auto& input_product : input_assembly_shipment.products)
    {
      output_assembly_shipment.products.push_back(
          convert_ros_product_to_core_product(input_product, briefcase_frame_id));
    }
    new_order.assembly_shipments.push_back(output_assembly_shipment);
  }
  orders_backlog_.push_back(new_order);
}

void ROSProcessManagement::agv1StateCallback(std_msgs::String::ConstPtr msg)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  agv_state_data_[AgvId::agv1] = msg->data;
}

void ROSProcessManagement::agv2StateCallback(std_msgs::String::ConstPtr msg)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  agv_state_data_[AgvId::agv2] = msg->data;
}

void ROSProcessManagement::agv3StateCallback(std_msgs::String::ConstPtr msg)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  agv_state_data_[AgvId::agv3] = msg->data;
}

void ROSProcessManagement::agv4StateCallback(std_msgs::String::ConstPtr msg)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  agv_state_data_[AgvId::agv4] = msg->data;
}

void ROSProcessManagement::agv1StationCallback(std_msgs::String::ConstPtr msg)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  if (validateStationId(msg->data))
  {
    agv_station_data_[AgvId::agv1] = tijcore::station_id::fromString(msg->data);
  }
}

void ROSProcessManagement::agv2StationCallback(std_msgs::String::ConstPtr msg)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  if (validateStationId(msg->data))
  {
    agv_station_data_[AgvId::agv2] = tijcore::station_id::fromString(msg->data);
  }
}

void ROSProcessManagement::agv3StationCallback(std_msgs::String::ConstPtr msg)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  if (validateStationId(msg->data))
  {
    agv_station_data_[AgvId::agv3] = tijcore::station_id::fromString(msg->data);
  }
}

void ROSProcessManagement::agv4StationCallback(std_msgs::String::ConstPtr msg)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  if (validateStationId(msg->data))
  {
    agv_station_data_[AgvId::agv4] = tijcore::station_id::fromString(msg->data);
  }
}

}  // namespace tijros
