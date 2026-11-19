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

#ifndef CM_TEST_SUITE__RUNNER_HPP_
#define CM_TEST_SUITE__RUNNER_HPP_

#include <chrono>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "cm_test_suite/cm_under_test.hpp"
#include "cm_test_suite/script.hpp"

namespace cm_test_suite
{
enum class Mode
{
  /// The harness cycles only while an operation runs and to settle afterwards.
  Stepped,
  /// A separate thread cycles all the time, like the real control loop.
  Threaded,
};

/// Failure categories, also used to skip checks. Events are categorized as event:<kind slug>,
/// so a single kind or all events can be skipped.
namespace check
{
constexpr char EVENT[] = "event";
constexpr char CONSISTENCY[] = "consistency";
constexpr char STATUS[] = "status";
constexpr char ACTIVITY[] = "activity";
constexpr char OUTCOME[] = "outcome";
constexpr char POWER_CUT[] = "power_cut";
constexpr char POWER_RECOVERY[] = "power_recovery";
constexpr char LIVENESS[] = "liveness";
constexpr char TIMEOUT[] = "timeout";
constexpr char RT_ALLOCATION[] = "rt_allocation";
constexpr char RT_BLOCKING[] = "rt_blocking";
constexpr char HANG[] = "hang";
constexpr char EXCEPTION[] = "exception";
constexpr char SETUP[] = "setup";
constexpr char SHUTDOWN[] = "shutdown";
constexpr char SETTLE[] = "settle";
}  // namespace check

/// Hard gates count right away. A provisional finding counts unless the referee rules it a suite
/// bug, because its oracle bound or time limit may itself be wrong.
enum class Tier
{
  Gate,
  Provisional,
};

Tier tier_of(const std::string & category);
const char * to_string(Tier tier);

#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
/// How soon after a power cut the controllers on the faulting hardware must be inactive.
constexpr std::chrono::milliseconds POWER_CUT_REACTION{2000};
/// How much later than its timeout a switch or a shutdown may return.
constexpr std::chrono::milliseconds TIMEOUT_MARGIN{2500};
#else
constexpr std::chrono::milliseconds POWER_CUT_REACTION{400};
constexpr std::chrono::milliseconds TIMEOUT_MARGIN{500};
#endif

struct RunOptions
{
  Mode mode = Mode::Stepped;
  Capabilities capabilities;
  CmConfig config;
  std::set<std::string> skipped_checks;
  /// Only report hard gate failures, as for judging.
  bool gates_only = false;
};

CmConfig default_config();

/// What a run exercised, so a script that never reaches the interesting states shows up.
struct RunStats
{
  struct Answers
  {
    size_t accepted = 0;
    size_t rejected = 0;
  };
  /// By step kind, such as "switch auto" or "load".
  std::map<std::string, Answers> answers;
  /// By hook and fault, such as "update:error".
  std::map<std::string, size_t> faults_armed;
  std::map<std::string, size_t> faults_taken;
  /// Steps during which the power was cut.
  size_t unpowered_steps = 0;
  /// Settling that ended at its deadline instead of in a quiet state.
  size_t settle_deadline_hits = 0;
  /// The longest of the cycles whose time is checked.
  std::chrono::nanoseconds longest_cycle{};
};

struct RunResult
{
  std::vector<std::string> failures;
  /// The executed steps with their outcome, up to the failing one.
  std::vector<std::string> log;
  RunStats stats;

  bool passed() const { return failures.empty(); }
};

/// Runs a script against a fresh controller manager. Stops at the first failing step.
RunResult run_script(const CmFactory & factory, const Script & script, const RunOptions & options);

/// Creates a controller manager with a description it must reject as a whole: no hardware of it
/// may be configured. Creating it may throw instead. Then shuts it down.
RunResult run_rejected_description(
  const CmFactory & factory, const std::string & description, const RunOptions & options);

}  // namespace cm_test_suite

#endif  // CM_TEST_SUITE__RUNNER_HPP_
