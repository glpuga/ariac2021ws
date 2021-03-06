/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// Standard library
#include <memory>
#include <mutex>
#include <thread>

// tijcore
#include <tijcore/utils/Timer.hpp>

// standard library
#include <chrono>
#include <mutex>

namespace tijcore {

namespace utils {

Timer::Timer(std::function<void()> callback) : callback_{callback} {}

Timer::~Timer() { stop(); }

void Timer::start(const std::chrono::microseconds &interval,
                  const bool repeat) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (runner_) {
    if (autoremove_) {
      // can be called with the mutex taken because the
      // runner thread has already terminated (us having the
      // mutex guarantees that)
      joinAndRemoveRunner();
    } else {
      throw std::logic_error("Can't start timer when timer is already started");
    }
  }
  interval_ = interval;
  repeat_ = repeat;
  createRunner();
}

void Timer::stop() {
  std::unique_lock<std::mutex> lock(mutex_);
  if (runner_) {
    halting_ = true;
    cv_.notify_all();
    lock.unlock();
    // this needs to be called with the mutex unlocked
    // to allow the thread to terminate
    joinAndRemoveRunner();
    lock.lock();
  }
}

bool Timer::isStarted() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return runner_ == nullptr;
}

void Timer::createRunner() {
  halting_ = false;
  autoremove_ = false;
  runner_ = std::make_unique<std::thread>([this]() { runnerBody(); });
}

void Timer::joinAndRemoveRunner() {
  runner_->join();
  runner_.reset();
}

void Timer::runnerBody() {
  std::unique_lock<std::mutex> lock(mutex_);
  do {
    cv_.wait_for(lock, interval_, [this]() { return halting_; });
    if (!halting_) {
      callback_();
    }
  } while (!halting_ && repeat_);
  autoremove_ = true;
}

} // namespace utils

} // namespace tijcore
