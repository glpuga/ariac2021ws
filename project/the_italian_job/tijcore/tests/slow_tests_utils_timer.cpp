/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library
#include <chrono>
#include <thread>

// gtest
#include "gmock/gmock.h"
#include "gtest/gtest.h"

// tijcore
#include <tijcore/utils/Timer.hpp>

namespace tijcore {

namespace utils {

namespace test {

namespace {

using testing::InSequence;

using namespace std::literals;

class HelperInterface {
public:
  virtual void event_1() = 0;
  virtual void event_2() = 0;
  virtual void event_3() = 0;
};

class HelperMock : public HelperInterface {
public:
  MOCK_METHOD0(event_1, void());
  MOCK_METHOD0(event_2, void());
  MOCK_METHOD0(event_3, void());
};

using ::testing::Test;

class TimerTests : public Test {
protected:
  const std::chrono::milliseconds base_interval_{100ms};
};

TEST_F(TimerTests, ConstructionTest) {
  // On construction the timer defaults to being off
  // and the callback should not get called
  HelperMock hm;
  EXPECT_CALL(hm, event_1()).Times(0);
  Timer uut{[&hm]() { hm.event_1(); }};
  std::this_thread::sleep_for(base_interval_);
}

TEST_F(TimerTests, StartWithOnlyOneParameter) {
  // With only one parameter start causes the
  // callback to get called repeatedly
  HelperMock hm;
  EXPECT_CALL(hm, event_1()).Times(3);
  Timer uut{[&hm]() { hm.event_1(); }};
  uut.start(2 * base_interval_);
  std::this_thread::sleep_for(7 * base_interval_);
}

TEST_F(TimerTests, StartWithRepeatTrue) {
  // With repeat set to true start causes the
  // callback to get called repeatedly
  HelperMock hm;
  EXPECT_CALL(hm, event_1()).Times(3);
  Timer uut{[&hm]() { hm.event_1(); }};
  uut.start(2 * base_interval_);
  std::this_thread::sleep_for(7 * base_interval_);
}

TEST_F(TimerTests, StartWithRepeatFalse) {
  // With repeat set to false the timer is only
  // shot once
  HelperMock hm;
  EXPECT_CALL(hm, event_1()).Times(1);
  Timer uut{[&hm]() { hm.event_1(); }};
  uut.start(2 * base_interval_, false);
  std::this_thread::sleep_for(7 * base_interval_);
}

TEST_F(TimerTests, StopAbortsARepeatingTimer) {
  // Stop prevents a timer from generating any further
  // activations of the callback
  HelperMock hm;
  EXPECT_CALL(hm, event_1()).Times(1);
  Timer uut{[&hm]() { hm.event_1(); }};
  uut.start(2 * base_interval_);

  // stop the timer 3 seconds into the run
  std::this_thread::sleep_for(3 * base_interval_);
  uut.stop();
  std::this_thread::sleep_for(4 * base_interval_);
}

TEST_F(TimerTests, StopAbortsARepeatingTimerBeforeDeadline) {
  // If called before the first execution of the a repeating timer
  // stop prevents the callback from being ever activated on a repeating timer
  HelperMock hm;
  EXPECT_CALL(hm, event_1()).Times(0);
  Timer uut{[&hm]() { hm.event_1(); }};
  uut.start(2 * base_interval_);
  // stop the timer 1 seconds into the run, one before the deadline
  std::this_thread::sleep_for(base_interval_);
  uut.stop();
  std::this_thread::sleep_for(4 * base_interval_);
}

TEST_F(TimerTests, StopAbortsANonRepeatingTimerBeforeDeadline) {
  // If called before the first execution of the a repeating timer
  // stop prevents the callback from being ever activated on a non repeating
  // timer
  HelperMock hm;
  EXPECT_CALL(hm, event_1()).Times(0);
  Timer uut{[&hm]() { hm.event_1(); }};
  uut.start(2 * base_interval_, false);
  // stop the timer 1 seconds into the run, one before the deadline
  std::this_thread::sleep_for(base_interval_);
  uut.stop();
  std::this_thread::sleep_for(3 * base_interval_);
}

TEST_F(TimerTests, StopDoesNothingToANonRepeatingTimerAfterDeadline) {
  // Calling stop after the deadline of a non-repeatable timer
  // does nothing.
  HelperMock hm;
  EXPECT_CALL(hm, event_1()).Times(1);
  Timer uut{[&hm]() { hm.event_1(); }};
  uut.start(base_interval_, false);
  // stop the timer 2 seconds into the run, one after the deadline
  std::this_thread::sleep_for(2 * base_interval_);
  uut.stop();
  std::this_thread::sleep_for(2 * base_interval_);
}

TEST_F(TimerTests, CannotRestartARepeatableTimer) {
  // Trying to start twice a repeatable timer throws
  Timer uut{[]() {}};
  EXPECT_NO_THROW({ uut.start(base_interval_); });
  EXPECT_THROW({ uut.start(base_interval_); }, std::logic_error);
}

TEST_F(TimerTests, CannotRestartANonRepeatableTimerBeforeDeadline) {
  // Trying to start twice a non-repeatable timer throws if
  // the second start happens before the activation of the timer
  Timer uut{[]() {}};
  EXPECT_NO_THROW({ uut.start(2 * base_interval_, false); });
  std::this_thread::sleep_for(base_interval_);
  EXPECT_THROW({ uut.start(base_interval_); }, std::logic_error);
}

TEST_F(TimerTests, RestartingANonRepeatableTimerAfterDeadlineStartsOver) {
  // Starting a non-repeatable timer with no stop() in between
  // does not fail if the second start is after the first activation
  // was carried out
  HelperMock hm;
  EXPECT_CALL(hm, event_1()).Times(2);
  Timer uut{[&hm]() { hm.event_1(); }};
  uut.start(base_interval_, false);
  std::this_thread::sleep_for(2 * base_interval_);
  uut.start(base_interval_, false);
  std::this_thread::sleep_for(2 * base_interval_);
}

TEST_F(TimerTests, RestartingRepeatableTimer) {
  // A repeatable timer can be started again if it's previously
  // stopped
  InSequence seq;
  HelperMock hm;
  EXPECT_CALL(hm, event_1()).Times(2);
  EXPECT_CALL(hm, event_2()).Times(2);
  EXPECT_CALL(hm, event_1()).Times(1);
  Timer uut{[&hm]() { hm.event_1(); }};
  uut.start(2 * base_interval_);
  std::this_thread::sleep_for(5 * base_interval_);
  uut.stop();
  // Here we use event_2() calls and InSequence mocked expectations
  // to ensure that the callback is not activated with the timer stopped.
  hm.event_2();
  std::this_thread::sleep_for(4 * base_interval_);
  hm.event_2();
  // Start the timer again
  uut.start(4 * base_interval_, false);
  std::this_thread::sleep_for(5 * base_interval_);
  uut.stop();
}

TEST_F(TimerTests, ChorusOfTimers) {
  // Checks that multiple timers with different periodicities
  // activate in the expected sequence when started at the same time
  InSequence seq;
  HelperMock hm;

  EXPECT_CALL(hm, event_3()).Times(1); // 3*base_interval_
  EXPECT_CALL(hm, event_2()).Times(1); // 5*base_interval_
  EXPECT_CALL(hm, event_3()).Times(1); // 6*base_interval_
  EXPECT_CALL(hm, event_1()).Times(1); // 7*base_interval_
  EXPECT_CALL(hm, event_3()).Times(1); // 9*base_interval_
  EXPECT_CALL(hm, event_2()).Times(1); // 10*base_interval_

  Timer uut1{[&hm]() { hm.event_1(); }};
  Timer uut2{[&hm]() { hm.event_2(); }};
  Timer uut3{[&hm]() { hm.event_3(); }};

  uut1.start(7 * base_interval_);
  uut2.start(5 * base_interval_);
  uut3.start(3 * base_interval_);
  std::this_thread::sleep_for(11 * base_interval_);
  uut1.stop();
  uut2.stop();
  uut3.stop();
}

TEST_F(TimerTests, TimerCanChangeModeBackAndForth) {
  // The operating mode of the timer can be changed on
  // each start
  InSequence seq;
  HelperMock hm;

  // expectation for sequence 1
  EXPECT_CALL(hm, event_1()).Times(2);
  // separator events
  EXPECT_CALL(hm, event_2()).Times(2);
  // expectation for sequence 2
  EXPECT_CALL(hm, event_1()).Times(1);
  // separator events
  EXPECT_CALL(hm, event_2()).Times(2);
  // expectation for sequence 3
  EXPECT_CALL(hm, event_1()).Times(2);

  Timer uut{[&hm]() { hm.event_1(); }};

  // sequence 1, repeatable timer
  uut.start(2 * base_interval_);
  std::this_thread::sleep_for(5 * base_interval_);
  uut.stop();

  hm.event_2(); // marker event
  std::this_thread::sleep_for(2 * base_interval_);
  hm.event_2(); // marker event

  // sequence 2, non-repeatable timer
  uut.start(2 * base_interval_, false);
  std::this_thread::sleep_for(5 * base_interval_);
  uut.stop();

  hm.event_2(); // marker event
  std::this_thread::sleep_for(2 * base_interval_);
  hm.event_2(); // marker event

  // sequence 3, repeatable timer
  uut.start(2 * base_interval_);
  std::this_thread::sleep_for(5 * base_interval_);
  uut.stop();
}

} // namespace

} // namespace test

} // namespace utils

} // namespace tijcore
