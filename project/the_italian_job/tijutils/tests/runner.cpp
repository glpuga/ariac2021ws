/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// gtest and gmock
#include "gtest/gtest.h"

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
