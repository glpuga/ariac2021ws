/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// gtest
#include "gtest/gtest.h"

// tijcore
#include "mocks/ProcessManagementMock.hpp"

namespace tijcore {

namespace test {

namespace {

using ::testing::Test;

class ProcessManagementMockTests : public Test {
public:
};

TEST_F(ProcessManagementMockTests, TheMockCanBeConstructed) {
  // just testing that the mock builds
  ProcessManagementMock uut;
}

} // namespace

} // namespace test

} // namespace tijcore
