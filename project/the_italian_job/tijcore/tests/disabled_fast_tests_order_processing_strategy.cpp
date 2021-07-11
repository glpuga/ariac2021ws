/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// gtest
#include "gtest/gtest.h"

// tijcore
#include "mocks/ResourceManagerMock.hpp"
#include <tijcore/perception/OrderProcessingStrategy.hpp>

namespace tijcore {

namespace test {

namespace {

using ::testing::Test;

class OrderProcessingStrategyTests : public Test {
public:
  const std::string test_order_id_{"order_1"};

  ResourceManagerMock::SharedPtr resource_manager_mock_;

  void SetUp() override {
    resource_manager_mock_ = std::make_unique<ResourceManagerMock>();
  }
};

}; // namespace

TEST_F(OrderProcessingStrategyTests, ObjectCanBeConstructed) {
  // just testing that the mock builds
  OrderProcessingStrategy uut;
}

TEST_F(OrderProcessingStrategyTests, CollectAgvsAndStationsInUse) {
  // just testing that the mock builds
  OrderProcessingStrategy uut;

  Order test_order;
  test_order.order_id = test_order_id_;

  {
    test_order.kitting_shipments.clear();
    test_order.kitting_shipments.push_back(
        KittingShipment{"shipment_type_id_1", AgvId::agv1, StationId::as1, {}});
    test_order.kitting_shipments.push_back(
        KittingShipment{"shipment_type_id_2", AgvId::agv3, StationId::as4, {}});
    const std::set<AgvId> expected_agvs_in_use{AgvId::agv1, AgvId::agv2,
                                               AgvId::agv3};
    const std::set<StationId> expected_stations_in_use{
        StationId::as1, StationId::as2, StationId::as4};
    std::set<AgvId> agvs_in_use{AgvId::agv2};
    std::set<StationId> stations_in_use{StationId::as2};
    uut.getAgvsAndStationsInUse(test_order, agvs_in_use, stations_in_use);
    ASSERT_EQ(expected_agvs_in_use, agvs_in_use);
    ASSERT_EQ(expected_stations_in_use, stations_in_use);
  }

  {
    test_order.kitting_shipments.clear();
    test_order.kitting_shipments.push_back(
        KittingShipment{"shipment_type_id_1", AgvId::any, StationId::as1, {}});
    test_order.kitting_shipments.push_back(
        KittingShipment{"shipment_type_id_2", AgvId::agv3, StationId::any, {}});
    const std::set<AgvId> expected_agvs_in_use{AgvId::agv2, AgvId::agv3};
    const std::set<StationId> expected_stations_in_use{StationId::as1,
                                                       StationId::as2};
    std::set<AgvId> agvs_in_use{AgvId::agv2};
    std::set<StationId> stations_in_use{StationId::as2};
    uut.getAgvsAndStationsInUse(test_order, agvs_in_use, stations_in_use);
    ASSERT_EQ(expected_agvs_in_use, agvs_in_use);
    ASSERT_EQ(expected_stations_in_use, stations_in_use);
  }
}

TEST_F(OrderProcessingStrategyTests, DisambiguateOrderWorks) {
  // just testing that the mock builds
  OrderProcessingStrategy uut;

  Order test_order;
  test_order.order_id = test_order_id_;
  test_order.kitting_shipments.push_back(
      KittingShipment{"shipment_type_id", AgvId::agv1, StationId::as1, {}});

  struct DisambiguationTestEntry {
    AgvId input_agv_id;
    StationId input_station_id;
    std::set<AgvId> input_agvs_in_use;
    std::set<StationId> input_stations_in_use;
    AgvId output_agv_id;
    StationId output_station_id;
  };

  const std::vector<DisambiguationTestEntry> test_cases = {
      {AgvId::agv1, StationId::as1, std::set<AgvId>{AgvId::agv1, AgvId::agv4},
       std::set<StationId>{StationId::as1, StationId::as4}, AgvId::agv1,
       StationId::as1},
      {AgvId::agv2, StationId::as1, std::set<AgvId>{AgvId::agv2, AgvId::agv4},
       std::set<StationId>{StationId::as1, StationId::as4}, AgvId::agv2,
       StationId::as1},
      {AgvId::agv3, StationId::as1, std::set<AgvId>{AgvId::agv3, AgvId::agv4},
       std::set<StationId>{StationId::as1, StationId::as4}, AgvId::agv3,
       StationId::as1},
      {AgvId::agv4, StationId::as1, std::set<AgvId>{AgvId::agv2, AgvId::agv4},
       std::set<StationId>{StationId::as1, StationId::as4}, AgvId::agv4,
       StationId::as1},
      // ---
      {AgvId::any, StationId::as1, std::set<AgvId>{AgvId::agv1, AgvId::agv4},
       std::set<StationId>{StationId::as1, StationId::as4}, AgvId::agv2,
       StationId::as1},
      {AgvId::any, StationId::as1, std::set<AgvId>{AgvId::agv2, AgvId::agv4},
       std::set<StationId>{StationId::as1, StationId::as4}, AgvId::agv1,
       StationId::as1},
      {AgvId::any, StationId::as2, std::set<AgvId>{AgvId::agv2, AgvId::agv4},
       std::set<StationId>{StationId::as2, StationId::as4}, AgvId::agv1,
       StationId::as2},
      {AgvId::any, StationId::as2, std::set<AgvId>{AgvId::agv1, AgvId::agv4},
       std::set<StationId>{StationId::as2, StationId::as4}, AgvId::agv2,
       StationId::as2},
      // ---
      {AgvId::any, StationId::as3, std::set<AgvId>{AgvId::agv3, AgvId::agv1},
       std::set<StationId>{StationId::as3, StationId::as1}, AgvId::agv4,
       StationId::as3},
      {AgvId::any, StationId::as3, std::set<AgvId>{AgvId::agv4, AgvId::agv1},
       std::set<StationId>{StationId::as3, StationId::as1}, AgvId::agv3,
       StationId::as3},
      {AgvId::any, StationId::as4, std::set<AgvId>{AgvId::agv4, AgvId::agv1},
       std::set<StationId>{StationId::as4, StationId::as1}, AgvId::agv3,
       StationId::as4},
      {AgvId::any, StationId::as4, std::set<AgvId>{AgvId::agv3, AgvId::agv1},
       std::set<StationId>{StationId::as4, StationId::as1}, AgvId::agv4,
       StationId::as4},
      // ---
      {AgvId::agv1, StationId::any, std::set<AgvId>{AgvId::agv1, AgvId::agv4},
       std::set<StationId>{StationId::as2, StationId::as4}, AgvId::agv1,
       StationId::as1},
      {AgvId::agv1, StationId::any, std::set<AgvId>{AgvId::agv1, AgvId::agv4},
       std::set<StationId>{StationId::as2, StationId::as4}, AgvId::agv1,
       StationId::as1},
      {AgvId::agv2, StationId::any, std::set<AgvId>{AgvId::agv2, AgvId::agv4},
       std::set<StationId>{StationId::as1, StationId::as4}, AgvId::agv2,
       StationId::as2},
      {AgvId::agv2, StationId::any, std::set<AgvId>{AgvId::agv2, AgvId::agv4},
       std::set<StationId>{StationId::as2, StationId::as4}, AgvId::agv2,
       StationId::as1},
      // ---
      {AgvId::agv3, StationId::any, std::set<AgvId>{AgvId::agv1, AgvId::agv3},
       std::set<StationId>{StationId::as1, StationId::as3}, AgvId::agv3,
       StationId::as4},
      {AgvId::agv3, StationId::any, std::set<AgvId>{AgvId::agv1, AgvId::agv3},
       std::set<StationId>{StationId::as1, StationId::as4}, AgvId::agv3,
       StationId::as3},
      {AgvId::agv4, StationId::any, std::set<AgvId>{AgvId::agv2, AgvId::agv4},
       std::set<StationId>{StationId::as1, StationId::as3}, AgvId::agv4,
       StationId::as4},
      {AgvId::agv4, StationId::any, std::set<AgvId>{AgvId::agv2, AgvId::agv4},
       std::set<StationId>{StationId::as1, StationId::as4}, AgvId::agv4,
       StationId::as3},
  };

  for (const auto &entry : test_cases) {
    test_order.kitting_shipments[0].agv_id = entry.input_agv_id;
    test_order.kitting_shipments[0].station_id = entry.input_station_id;
    auto agvs_in_use = entry.input_agvs_in_use;
    auto stations_in_use = entry.input_stations_in_use;

    ASSERT_TRUE(agv::isAny(entry.input_agv_id) ||
                agvs_in_use.count(entry.input_agv_id));
    ASSERT_TRUE(station_id::isAny(entry.input_station_id) ||
                stations_in_use.count(entry.input_station_id));

    uut.disambiguateOrderEndpoints(resource_manager_mock_, test_order,
                                   agvs_in_use, stations_in_use);
    ASSERT_EQ(entry.output_agv_id, test_order.kitting_shipments[0].agv_id);
    ASSERT_EQ(entry.output_station_id,
              test_order.kitting_shipments[0].station_id);

    ASSERT_TRUE(!agv::isAny(entry.output_agv_id) &&
                agvs_in_use.count(entry.output_agv_id));
    ASSERT_TRUE(!station_id::isAny(entry.output_station_id) &&
                stations_in_use.count(entry.output_station_id));
  }
}

} // namespace test

} // namespace tijcore
