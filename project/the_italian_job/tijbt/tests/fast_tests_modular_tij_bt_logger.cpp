/* Copyright [2022] <TheItalianJob>
 * Author: Gerardo Puga
 */

// gtest
#include "gtest/gtest.h"

// tijcore
#include <tijbt/modular_loggers/ModularTIJBTLogger.hpp>

namespace tijbt
{
namespace test
{
namespace
{
struct ModularTIJBTLoggerTests : public ::testing::Test
{
};

TEST_F(ModularTIJBTLoggerTests, TrivialConstructionTest)
{
  // this tests that the header files are ok, and that the object can be built without crashing
  ModularTIJBTLogger uut;
}

}  // namespace

}  // namespace test

}  // namespace tijbt
