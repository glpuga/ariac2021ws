/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// gtest
#include "gtest/gtest.h"

// tijcore
#include "mocks/BlindVolumeTrackerMock.hpp"

namespace tijcore
{
namespace test
{
namespace
{
struct BlindVolumeTrackerMockTests : public ::testing::Test
{
};

TEST_F(BlindVolumeTrackerMockTests, ConstructionTest)
{
  BlindVolumeTrackerMock uut;
}

}  // namespace

}  // namespace test

}  // namespace tijcore
