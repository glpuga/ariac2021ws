/* Copyright [2022] <TheItalianJob>
 * Author: Gerardo Puga
 */

// gtest
#include "gtest/gtest.h"

// tijcore
#include <tijbt/modular_loggers/ModularStdCoutLogger.hpp>

namespace tijbt
{
namespace test
{
namespace
{
struct ModularStdCoutLoggerTests : public ::testing::Test
{
};

TEST_F(ModularStdCoutLoggerTests, TrivialConstructionTest)
{
  // this tests that the header files are ok, and that the object can be built without crashing
  ModularStdCoutLogger uut;
}

}  // namespace

}  // namespace test

}  // namespace tijbt
