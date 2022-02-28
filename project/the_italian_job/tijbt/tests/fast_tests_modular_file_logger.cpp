/* Copyright [2022] <TheItalianJob>
 * Author: Gerardo Puga
 */

// gtest
#include "gtest/gtest.h"

// tijcore
#include <tijbt/modular_loggers/ModularFileLogger.hpp>

namespace tijbt
{
namespace test
{
namespace
{
struct ModularFileLoggerTests : public ::testing::Test
{
};

TEST_F(ModularFileLoggerTests, TrivialConstructionTest)
{
  // this tests that the header files are ok, and that the object can be built without crashing
  ModularFileLogger uut("testfilename");
}

}  // namespace

}  // namespace test

}  // namespace tijbt
