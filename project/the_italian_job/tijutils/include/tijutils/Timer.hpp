/* Copyright [2022] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

namespace tijutils
{
class Timer
{
public:
  explicit Timer(std::function<void()> callback);

  ~Timer();

  void start(const std::chrono::microseconds& interval, const bool repeat = true);

  void stop();

  bool isStarted() const;

private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::unique_ptr<std::thread> runner_;
  bool halting_{ false };
  bool autoremove_{ false };

  std::chrono::microseconds interval_;
  bool repeat_{ false };

  std::function<void()> callback_;

  void createRunner();
  void joinAndRemoveRunner();

  void runnerBody();
};

}  // namespace tijutils
