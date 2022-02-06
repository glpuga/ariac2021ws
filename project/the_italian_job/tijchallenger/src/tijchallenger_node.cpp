/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#include <ros/ros.h>

// project
#include <tijchallenger/TIJChallenger.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tijchallenger_node");
  tijchallenger::TIJChallenger challenger{};
  challenger.run();
  return 0;
}
