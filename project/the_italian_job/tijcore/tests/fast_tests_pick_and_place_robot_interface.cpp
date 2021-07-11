/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// gtest
#include "gtest/gtest.h"

// tijcore
#include "mocks/PickAndPlaceRobotMock.hpp"

namespace tijcore {

namespace test {

namespace {

using ::testing::Test;

class PickAndPlaceRobotInterfaceTests : public Test {
public:
};

TEST_F(PickAndPlaceRobotInterfaceTests, TheMockCanBeConstructed) {
  // just testing that the mock builds
  PickAndPlaceRobotMock uut;
}

} // namespace

} // namespace test

} // namespace tijcore
