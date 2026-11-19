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

#ifndef CM_TEST_SUITE__FAULTS_HPP_
#define CM_TEST_SUITE__FAULTS_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

#include "cm_test_suite/types.hpp"

namespace cm_test_suite
{
enum class Hook : uint8_t
{
  Read,
  Write,
  Prepare,
  Perform,
  Configure,
  Activate,
  Deactivate,
  Cleanup,
  Error,
  Update,
  ChainedMode,
  Shutdown,
  Count,
};

enum class Fault : uint8_t
{
  None,
  Error,
  Failure,
  Deactivate,
  Throw,
  Delay,
  Hang,
  /// The controller writes no command in its first updates after this activation.
  Quiet,
  /// A lifecycle callback that uses up nearly all of the switch timeout.
  Late,
};

/// A slow callback, well below every timeout.
constexpr std::chrono::milliseconds DELAY{20};
/// A command mode switch as slow as a Dynamixel bus round trip with verification, longer than a
/// control cycle may take.
constexpr std::chrono::milliseconds SLOW_SWITCH{30};
/// A callback that ends just before the default switch timeout of 1 s, so the deadline passes in
/// the slow prepare or perform that follows it.
constexpr std::chrono::milliseconds LATE{975};
/// How long each read and write of async hardware takes in a slow I/O step, and for how long.
constexpr std::chrono::milliseconds SLOW_IO{1};
constexpr std::chrono::milliseconds SLOW_IO_TIME{300};
/// The longest read or write of async hardware with jitter, five control periods of 1 ms.
constexpr std::chrono::milliseconds IO_JITTER{5};
/// A write of async hardware whose owner stalls, half of Hector's 100 ms hang rule.
constexpr std::chrono::milliseconds IO_STALL{50};
/// A stall just below Hector's 100 ms hang rule, so a hang window that counts more than the stall
/// faults a component that still makes progress.
constexpr std::chrono::milliseconds IO_LONG_STALL{80};
/// A callback that outlives the switch timeout of the harness.
constexpr std::chrono::milliseconds HANG{2500};
/// Updates without a command after a quiet activation, below the 5 cycle pause that needs a hold.
constexpr int QUIET_UPDATES = 3;

const char * to_string(Hook hook);
const char * to_string(Fault fault);
std::optional<Hook> parse_hook(std::string_view text);
std::optional<Fault> parse_fault(std::string_view text);

/// Whether a plugin can apply the fault at that hook.
bool applicable(TargetKind kind, Hook hook, Fault fault);

/// One shot faults the harness arms and the rig plugins take on their next matching call.
class FaultBoard
{
public:
  static constexpr size_t MAX_TARGETS = 32;

  /// Throws for a fault that is not applicable at the hook.
  void arm(Target target, Hook hook, Fault fault);
  /// Returns and disarms the fault armed for this call. Only error, failure, deactivate, throw,
  /// hang and late count as fired.
  Fault take(Target target, Hook hook);
  /// Whether a fault is armed for this call and not taken yet.
  bool pending(Target target, Hook hook) const;
  /// Disarms everything and forgets which faults fired.
  void clear();
  /// The targets whose fired faults were taken since the last call.
  std::vector<Target> take_fired();

private:
  static constexpr size_t SLOTS = 2 * MAX_TARGETS * static_cast<size_t>(Hook::Count);

  size_t slot(Target target, Hook hook) const;

  std::array<std::atomic<Fault>, SLOTS> armed_{};
  std::array<std::atomic<bool>, SLOTS> fired_{};
};

FaultBoard & faults();

}  // namespace cm_test_suite

#endif  // CM_TEST_SUITE__FAULTS_HPP_
