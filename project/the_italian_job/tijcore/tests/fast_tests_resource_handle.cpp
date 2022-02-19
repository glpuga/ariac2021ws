/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// standard library
#include <memory>

// gtest
#include "gtest/gtest.h"

// tijcore
#include <tijcore/resources/ResourceHandle.hpp>

namespace tijcore
{
namespace utils
{
namespace test
{
namespace
{
class SomeResourse
{
};

using ::testing::Test;

class InUseMarkHandlerTests : public Test
{
};

TEST_F(InUseMarkHandlerTests, SharedRegularUse)
{
  ResourceHandle<SomeResourse> uut{ std::make_shared<SomeResourse>() };

  ASSERT_FALSE(uut.allocated());
  {
    ResourceHandle<SomeResourse> user_copy = uut;
    ASSERT_TRUE(uut.allocated());
  }
  ASSERT_FALSE(uut.allocated());
}

TEST_F(InUseMarkHandlerTests, SharedNullResourceThrows)
{
  ASSERT_THROW(
      { ResourceHandle<SomeResourse> uut{ ResourceHandle<SomeResourse>::ResourceSharedPtr{} }; },
      std::invalid_argument);
}

TEST_F(InUseMarkHandlerTests, SharedUseCountNeedsToBeOneOnConstruction)
{
  auto resource = std::make_shared<SomeResourse>();
  auto resource_copy = resource;

  ASSERT_THROW({ ResourceHandle<SomeResourse> uut{ resource }; }, std::invalid_argument);
}

TEST_F(InUseMarkHandlerTests, ResourcePointerMethodConstAndNonConst)
{
  const ResourceHandle<SomeResourse> uut1{ std::make_shared<SomeResourse>() };
  ResourceHandle<SomeResourse> uut2{ std::make_shared<SomeResourse>() };
  ASSERT_NE(nullptr, uut1.resource());
  ASSERT_NE(nullptr, uut2.resource());
}

TEST_F(InUseMarkHandlerTests, UniqueRegularUse)
{
  ResourceHandle<SomeResourse> uut{ std::make_unique<SomeResourse>() };

  ASSERT_FALSE(uut.allocated());
  {
    ResourceHandle<SomeResourse> user_copy = uut;
    ASSERT_TRUE(uut.allocated());
  }
  ASSERT_FALSE(uut.allocated());
}

TEST_F(InUseMarkHandlerTests, UniqueNullResourceThrows)
{
  ASSERT_THROW(
      { ResourceHandle<SomeResourse> uut{ ResourceHandle<SomeResourse>::ResourceUniquePtr{} }; },
      std::invalid_argument);
}

}  // namespace

}  // namespace test

}  // namespace utils

}  // namespace tijcore
