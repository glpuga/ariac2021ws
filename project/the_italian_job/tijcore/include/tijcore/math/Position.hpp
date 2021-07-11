/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// tijcore
#include <tijcore/math/Vector3.hpp>

namespace tijcore {

class Position {
public:
  static const Position Zero;

  Position() = default;

  explicit Position(const Vector3 &position) : position_{position} {}

  Vector3 vector() const { return position_; }

  // TODO(glpuga) test this variant
  Vector3 &vector() { return position_; }

  static bool samePosition(const Position &lhs, const Position &rhs,
                           const double tolerance);

  static Position fromVector(const double x, const double y, const double z);

private:
  Vector3 position_;
};

std::ostream &operator<<(std::ostream &os, const Position &p);

} // namespace tijcore
