/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <ostream>

namespace tijmath
{
class Quaternion
{
public:
  Quaternion() = default;

  Quaternion(const double x, const double y, const double z, const double w)
    : x_{ x }, y_{ y }, z_{ z }, w_{ w }
  {
  }

  double x() const
  {
    return x_;
  }
  double y() const
  {
    return y_;
  }
  double z() const
  {
    return z_;
  }
  double w() const
  {
    return w_;
  }

  double& x()
  {
    return x_;
  }
  double& y()
  {
    return y_;
  }
  double& z()
  {
    return z_;
  }
  double& w()
  {
    return w_;
  }

private:
  double x_{ 0.0 };
  double y_{ 0.0 };
  double z_{ 0.0 };
  double w_{ 1.0 };
};

std::ostream& operator<<(std::ostream& os, const Quaternion& q);

}  // namespace tijmath
