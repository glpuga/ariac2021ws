/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */
#pragma once

// standard library
#include <deque>
#include <functional>
#include <utility>

namespace tijcore {

namespace test {

namespace utils {

class ActionQueue {
public:
  using ActionFunction = std::function<void()>;

  void queueTestActionQueue(ActionFunction &&f) {
    action_queue_.emplace_back(std::move(f));
  }

  bool runTestActionQueue() {
    try {
      while (!action_queue_.empty()) {
        auto &action = action_queue_.front();
        action();
        action_queue_.pop_front();
      }
    } catch (const std::exception &e) {
      std::cerr << "Unexpected exception thrown while executing action queue: "
                << e.what() << std::endl;
      return false;
    }
    return true;
  }

private:
  std::deque<ActionFunction> action_queue_;
};

} // namespace utils

} // namespace test

} // namespace tijcore
