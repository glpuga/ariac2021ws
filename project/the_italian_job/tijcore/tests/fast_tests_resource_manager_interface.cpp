/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// gtest
#include "gtest/gtest.h"

// tijcore
#include "mocks/ResourceManagerMock.hpp"

namespace tijcore {

namespace test {

namespace {

using ::testing::Test;

class ResourceManagerInterfaceTests : public Test {
public:
};

TEST_F(ResourceManagerInterfaceTests, TheMockCanBeConstructed) {
  // just testing that the mock builds
  ResourceManagerMock uut;
}

} // namespace

} // namespace test

} // namespace tijcore
