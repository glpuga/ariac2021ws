/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library
#include <iostream>
#include <sstream>

// gtest
#include "gtest/gtest.h"

// tijcore
#include <tijcore/agents/WorkRegionId.hpp>

namespace tijcore
{
namespace test
{
namespace
{
using ::testing::Test;

class WorkRegionIdTests : public Test
{
public:
};

TEST_F(WorkRegionIdTests, TestFromString)
{
  ASSERT_EQ(WorkRegionId::kitting_near_bins, work_region::fromString("kitting_near_bins"));
  ASSERT_EQ(WorkRegionId::kitting_far_bins, work_region::fromString("kitting_far_bins"));
  ASSERT_EQ(WorkRegionId::kitting_agvs, work_region::fromString("kitting_agvs"));
  ASSERT_EQ(WorkRegionId::assembly, work_region::fromString("assembly"));
  ASSERT_EQ(WorkRegionId::conveyor_belt, work_region::fromString("conveyor_belt"));
  ASSERT_THROW(work_region::fromString("kitchen"), std::invalid_argument);
}

TEST_F(WorkRegionIdTests, TestToString)
{
  ASSERT_EQ("kitting_near_bins", work_region::toString(WorkRegionId::kitting_near_bins));
  ASSERT_EQ("kitting_far_bins", work_region::toString(WorkRegionId::kitting_far_bins));
  ASSERT_EQ("kitting_agvs", work_region::toString(WorkRegionId::kitting_agvs));
  ASSERT_EQ("assembly", work_region::toString(WorkRegionId::assembly));
  ASSERT_EQ("conveyor_belt", work_region::toString(WorkRegionId::conveyor_belt));
}

TEST_F(WorkRegionIdTests, TestStreamOperator)
{
  std::ostringstream os;

  os << WorkRegionId::kitting_near_bins << "\n"
     << WorkRegionId::kitting_far_bins << "\n"
     << WorkRegionId::kitting_agvs << "\n"
     << WorkRegionId::assembly << "\n"
     << WorkRegionId::conveyor_belt << "\n";

  ASSERT_EQ(
      "kitting_near_bins\n"
      "kitting_far_bins\n"
      "kitting_agvs\n"
      "assembly\n"
      "conveyor_belt\n",
      os.str());
}

TEST_F(WorkRegionIdTests, TestIsValid)
{
  ASSERT_TRUE(work_region::isValid("kitting_near_bins"));
  ASSERT_TRUE(work_region::isValid("kitting_far_bins"));
  ASSERT_TRUE(work_region::isValid("kitting_agvs"));
  ASSERT_TRUE(work_region::isValid("assembly"));
  ASSERT_TRUE(work_region::isValid("conveyor_belt"));

  ASSERT_FALSE(work_region::isValid(""));
  ASSERT_FALSE(work_region::isValid("kitchen"));
  ASSERT_FALSE(work_region::isValid("europe"));
  ASSERT_FALSE(work_region::isValid("foo"));
}

}  // namespace

}  // namespace test

}  // namespace tijcore
