// Copyright (2021) Gerardo Puga
// Distributed under the MIT License (http://opensource.org/licenses/MIT)

// Standard library
#include <iostream>
#include <set>
#include <sstream>
#include <unordered_set>

// gtest
#include "gtest/gtest.h"

// project
#include <tijutils/UniqueId.hpp>

namespace tijutils
{
namespace test
{
namespace
{
using ::testing::Test;

struct UniqueIdTest : public Test
{
};

TEST_F(UniqueIdTest, UniquenessTest)
{
  const auto unique_id_1 = UniqueId::CreateNewId();
  const auto unique_id_2 = UniqueId::CreateNewId();
  const auto unique_id_3 = UniqueId::CreateNewId();

  ASSERT_EQ(unique_id_1, unique_id_1);
  ASSERT_EQ(unique_id_2, unique_id_2);
  ASSERT_EQ(unique_id_3, unique_id_3);

  ASSERT_NE(unique_id_1, unique_id_2);
  ASSERT_NE(unique_id_1, unique_id_3);

  ASSERT_NE(unique_id_2, unique_id_1);
  ASSERT_NE(unique_id_2, unique_id_3);

  ASSERT_NE(unique_id_3, unique_id_1);
  ASSERT_NE(unique_id_3, unique_id_2);
}

TEST_F(UniqueIdTest, TestFromString)
{
  const auto uut = UniqueId::CreateNewId();

  std::ostringstream os;
  os << std::hex << uut.value();

  ASSERT_EQ(os.str(), unique_id::toString(uut));
}

TEST_F(UniqueIdTest, OrderedAndHashable)
{
  // if this test builds it checks that UniqueId is both hashable and sortable
  {
    std::set<UniqueId> uut;
    uut.insert(UniqueId::CreateNewId());
    uut.insert(UniqueId::CreateNewId());
  }
  {
    std::unordered_set<UniqueId> uut;
    uut.insert(UniqueId::CreateNewId());
    uut.insert(UniqueId::CreateNewId());
  }
}

}  // namespace

}  // namespace test

}  // namespace tijutils
