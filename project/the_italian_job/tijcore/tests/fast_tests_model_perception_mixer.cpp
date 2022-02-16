/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// standard library
#include <memory>
#include <utility>
#include <vector>

// gtest
#include "gmock/gmock.h"
#include "gtest/gtest.h"

// tijcore
#include <tijcore/coremodels/ModelPerceptionMixer.hpp>

// mocks
#include "mocks/ModelPerceptionMixerMock.hpp"

namespace tijcore
{
namespace utils
{
namespace test
{
namespace
{
using ::testing::Return;
using ::testing::Test;

class ModelPerceptionMixerTests : public Test
{
public:
  using VectorOfModels = std::vector<ObservedModel>;
};

TEST_F(ModelPerceptionMixerTests, EmptyItemsInVector)
{
  auto mock_camera_0 = std::make_unique<ModelPerceptionMock>();
  VectorOfModels camera_0_0 = {
    { PartId("assembly_pump_red"), {} },
    { PartId("assembly_pump_blue"), {} },
  };
  VectorOfModels camera_0_1 = {
    { PartId("assembly_sensor_blue"), {} },
  };
  VectorOfModels camera_0_2 = {};
  VectorOfModels camera_0_3 = {};

  EXPECT_CALL(*mock_camera_0, getObservedModels())
      .WillOnce(Return(camera_0_0))
      .WillOnce(Return(camera_0_1))
      .WillOnce(Return(camera_0_2))
      .WillOnce(Return(camera_0_3));

  auto mock_camera_1 = std::make_unique<ModelPerceptionMock>();
  VectorOfModels camera_1_0 = {
    { PartId("assembly_battery_red"), {} },
    { PartId("assembly_regulator_blue"), {} },
  };
  VectorOfModels camera_1_1 = {};
  VectorOfModels camera_1_2 = {
    { PartId("assembly_regulator_blue"), {} },
  };
  VectorOfModels camera_1_3 = {};

  EXPECT_CALL(*mock_camera_1, getObservedModels())
      .WillOnce(Return(camera_1_0))
      .WillOnce(Return(camera_1_1))
      .WillOnce(Return(camera_1_2))
      .WillOnce(Return(camera_1_3));

  std::vector<ModelPerceptionInterface::Ptr> cameras;
  cameras.push_back(std::move(mock_camera_0));
  cameras.push_back(std::move(mock_camera_1));
  auto uut = std::make_unique<ModelPerceptionMixer>(std::move(cameras));

  VectorOfModels expected_models_0 = {
    { PartId("assembly_pump_red"), {} },
    { PartId("assembly_pump_blue"), {} },
    { PartId("assembly_battery_red"), {} },
    { PartId("assembly_regulator_blue"), {} },
  };
  VectorOfModels expected_models_1 = {
    { PartId("assembly_sensor_blue"), {} },
  };
  VectorOfModels expected_models_2 = {
    { PartId("assembly_regulator_blue"), {} },
  };
  VectorOfModels expected_models_3 = {};

  const auto part_ids_match = [](const auto& v1, const auto& v2) {
    if (v1.size() != v2.size())
    {
      return false;
    }
    for (auto it1 = v1.cbegin(), it2 = v2.cbegin(); it1 != v1.end(); ++it1, ++it2)
    {
      if (it1->type != it2->type)
      {
        return false;
      }
    }
    return true;
  };

  ASSERT_TRUE(part_ids_match(expected_models_0, uut->getObservedModels()));
  ASSERT_TRUE(part_ids_match(expected_models_1, uut->getObservedModels()));
  ASSERT_TRUE(part_ids_match(expected_models_2, uut->getObservedModels()));
  ASSERT_TRUE(part_ids_match(expected_models_3, uut->getObservedModels()));
}

}  // namespace

}  // namespace test

}  // namespace utils

}  // namespace tijcore
