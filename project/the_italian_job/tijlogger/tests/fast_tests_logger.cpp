/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// gtest
#include "gtest/gtest.h"

// project
#include <tijlogger/logger.hpp>

namespace logger
{
namespace test
{
namespace
{
using ::testing::Test;

class WorkRegasdfaionIdTests : public Test
{
};

TEST_F(WorkRegasdfaionIdTests, TestFromString)
{
  ERROR("{} {} {}", "hello world!", 1, 2);
  WARNING("{} {} {}", "hello world!", 1, 2);
  INFO("{} {} {}", "hello world!", 1, 2);
  DEBUG("{} {} {}", "hello world!", 1, 2);
}

TEST_F(WorkRegasdfaionIdTests, TestFroedmString)
{
  logger::instance().setLevel(logger::Level::Debug);
  ERROR("{} {} {}", "hello world!", 1, 2);
  WARNING("{} {} {}", "hello world!", 1, 2);
  INFO("{} {} {}", "hello world!", 1, 2);
  DEBUG("{} {} {}", "hello world!", 1, 2);
}

}  // namespace

}  // namespace test

}  // namespace logger
