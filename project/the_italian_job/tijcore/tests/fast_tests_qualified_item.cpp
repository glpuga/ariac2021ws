/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// standard library
#include <string>

// gtest
#include "gtest/gtest.h"

// tijcore
#include <tijcore/datatypes/AnonymizedDataHolder.hpp>

namespace tijcore
{
namespace test
{
namespace
{
using ::testing::Test;

struct TypeA
{
  int data{ 0 };
};

struct TypeB
{
  std::string data;
};

struct QualifiedItemTests : public Test
{
};

TEST_F(QualifiedItemTests, DefaultConstructor)
{
  AnonymizedDataHolder uut;
  ASSERT_FALSE(uut.is<TypeA>());
  ASSERT_FALSE(uut.is<TypeB>());
}

TEST_F(QualifiedItemTests, TypeErasedConstructor)
{
  const TypeB expected_part{ "some data" };
  const AnonymizedDataHolder uut{ expected_part };
  ASSERT_FALSE(uut.is<TypeA>());
  ASSERT_TRUE(uut.is<TypeB>());
  const auto& recovered_part = uut.as<TypeB>();
  ASSERT_EQ(recovered_part.data, expected_part.data);
}

TEST_F(QualifiedItemTests, NonConstAccessToData)
{
  const TypeA original_content{ 42 };
  AnonymizedDataHolder uut{ original_content };
  const auto& recovered_contents = uut.as<TypeA>();
  ASSERT_EQ(recovered_contents.data, original_content.data);
  const TypeA new_content{ 1555 };
  uut.as<TypeA>().data = new_content.data;
  ASSERT_EQ(recovered_contents.data, new_content.data);
}

TEST_F(QualifiedItemTests, IdenticalTypeAssignment)
{
  const TypeA original_content{ 42 };
  AnonymizedDataHolder uut{ original_content };
  {
    const auto& recovered_contents = uut.as<TypeA>();
    ASSERT_EQ(recovered_contents.data, original_content.data);
  }
  const TypeB new_content{ "new contents" };
  uut = new_content;
  {
    const auto& recovered_contents = uut.as<TypeB>();
    ASSERT_EQ(recovered_contents.data, new_content.data);
  }
}

TEST_F(QualifiedItemTests, DirectContainedTypeAssignment)
{
  const TypeA original_content{ 42 };
  AnonymizedDataHolder uut{ original_content };
  {
    const auto& recovered_contents = uut.as<TypeA>();
    ASSERT_EQ(recovered_contents.data, original_content.data);
  }

  {
    const TypeA new_content{ 33 };
    uut = new_content;
    const auto& recovered_contents = uut.as<TypeA>();
    ASSERT_EQ(recovered_contents.data, new_content.data);
  }

  {
    const TypeB new_content{ "hello world" };
    uut = new_content;
    const auto& recovered_contents = uut.as<TypeB>();
    ASSERT_EQ(recovered_contents.data, new_content.data);
  }
}

}  // namespace

}  // namespace test

}  // namespace tijcore
