// Copyright 2026 Team Hector
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CONTROL_LOOP_HPP_
#define CONTROL_LOOP_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "cm_test_suite/cm_under_test.hpp"
#include "cm_test_suite/runner.hpp"

namespace cm_test_suite
{
/// Calls the controller manager's cycle, either on demand or from its own thread.
class ControlLoop
{
public:
  using Clock = std::chrono::steady_clock;

  ControlLoop(CmUnderTest & cm, Mode mode);
  ~ControlLoop();

  ControlLoop(const ControlLoop &) = delete;
  ControlLoop & operator=(const ControlLoop &) = delete;

  /// Lets the loop progress a little while the harness waits for something.
  void tick();
  /// Returns once at least this many further cycles ran.
  void run_cycles(size_t cycles);
  /// Keeps the loop running for a while, but at least one cycle.
  /// No cycle runs for the duration or until `until` holds, as if the control loop got stuck.
  void stall(Clock::duration duration, const std::function<bool()> & until = [] { return false; });
  /// No cycle runs after this returns.
  void stop();

  /// Exceptions that escaped cycle().
  std::vector<std::string> take_errors();
  /// The longest cycle since the last call, without the time other threads preempted it.
  Clock::duration take_longest_cycle();

private:
  void cycle();

  CmUnderTest & cm_;
  const Mode mode_;
  std::atomic<bool> running_{true};
  std::atomic<bool> paused_{false};
  std::mutex errors_mutex_;
  std::vector<std::string> errors_;
  std::atomic<Clock::rep> longest_cycle_{0};
  std::thread thread_;
};

}  // namespace cm_test_suite

#endif  // CONTROL_LOOP_HPP_
