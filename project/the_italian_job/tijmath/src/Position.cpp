/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <stdexcept>

// tijmath
#include <tijmath/Position.hpp>

namespace tijmath
{
const Position Position::Zero{ Vector3{ 0, 0, 0 } };

bool Position::samePosition(const Position& lhs, const Position& rhs, const double tolerance)
{
  const auto distance = (lhs.vector() - rhs.vector()).norm();
  if (tolerance < 0.0)
  {
    throw std::invalid_argument{ "Position tolerance needs to be a positive number" };
  }
  return distance < tolerance;
}

Position Position::fromVector(const double x, const double y, const double z)
{
  return Position{ Vector3{ x, y, z } };
}

std::ostream& operator<<(std::ostream& os, const Position& p)
{
  os << p.vector();
  return os;
}

}  // namespace tijmath
