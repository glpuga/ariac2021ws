/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// Standard library

// tijcore
#include <tijcore/math/Quaternion.hpp>

namespace tijcore {

std::ostream &operator<<(std::ostream &os, const Quaternion &q) {
  return os << "[ " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
            << " ]";
}

} // namespace tijcore
