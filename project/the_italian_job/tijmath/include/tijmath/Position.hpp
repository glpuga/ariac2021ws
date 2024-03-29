/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// tijmath
#include <tijmath/Vector3.hpp>

namespace tijmath
{
class Position
{
public:
  static const Position Zero;

  Position() = default;

  explicit Position(const Vector3& position) : position_{ position }
  {
  }

  Vector3 vector() const
  {
    return position_;
  }

  // TODO(glpuga) test this variant
  Vector3& vector()
  {
    return position_;
  }

  static bool samePosition(const Position& lhs, const Position& rhs, const double tolerance);

  static Position fromVector(const double x, const double y, const double z);

private:
  Vector3 position_;
};

std::ostream& operator<<(std::ostream& os, const Position& p);

}  // namespace tijmath
