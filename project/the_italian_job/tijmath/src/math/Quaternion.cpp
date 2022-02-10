/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library

// tijmath
#include <tijmath/math/Quaternion.hpp>

namespace tijmath
{
std::ostream& operator<<(std::ostream& os, const Quaternion& q)
{
  return os << "[ " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " ]";
}

}  // namespace tijmath
