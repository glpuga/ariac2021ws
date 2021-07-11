/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// standard library
#include <mutex>

// tijcore
#include <tijcore/perception/InUseMarkImplementation.hpp>

namespace tijcore {

void InUseMarkImplementation::lock() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (in_use_) {
    throw std::logic_error("Trying to lock a resource that's already taken");
  }
  in_use_ = true;
}

void InUseMarkImplementation::unlock() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!in_use_) {
    throw std::logic_error("Trying to release a resource that's not taken");
  }
  in_use_ = false;
}

bool InUseMarkImplementation::isInUse() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return in_use_;
}

} // namespace tijcore
