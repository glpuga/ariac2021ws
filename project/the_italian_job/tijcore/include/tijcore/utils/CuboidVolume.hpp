/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// tijcore
#include <tijmath/Pose3.hpp>
#include <tijmath/Vector3.hpp>

namespace tijcore
{
class CuboidVolume
{
public:
  CuboidVolume();

  CuboidVolume(const tijmath::Vector3& lower_right_back, const tijmath::Vector3& upper_left_front);

  bool contains(const tijmath::Vector3& point) const;
  bool contains(const tijmath::Position& point) const;
  bool contains(const tijmath::Pose3& point) const;

  tijmath::Position lowerRightBackCorner() const;
  tijmath::Position upperLeftFrontCorner() const;

private:
  tijmath::Vector3 lower_right_back_;
  tijmath::Vector3 upper_left_front_;
};

}  // namespace tijcore
