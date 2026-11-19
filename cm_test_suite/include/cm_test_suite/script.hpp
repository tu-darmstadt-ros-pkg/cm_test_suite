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

#ifndef CM_TEST_SUITE__SCRIPT_HPP_
#define CM_TEST_SUITE__SCRIPT_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "cm_test_suite/cm_under_test.hpp"
#include "cm_test_suite/faults.hpp"
#include "cm_test_suite/trace.hpp"

/// Test scripts: what the harness does to the controller manager, step by step.
///
/// A script never depends on the outcome of earlier steps, so it can be generated from a seed,
/// written to text, replayed and shrunk. One step per line:
///
///   switch strict +teleop -nav fault=hardware:base:write:error@3 stall powercut@2 expect=accept
///   fault controller:arm_ik:update:throw [fault=controller:arm_ik:deactivate:hang ...] [nowait]
///   load nav | configure nav | cleanup nav | unload nav    [fault=...] [expect=...]
///   recover arm                                             [fault=...] [expect=...]
///   powercut on [nowait] | powercut off
///   slowio [jitter|stall|long_stall]
///   shutdown                                                [fault=...]
///
/// A switch fault is armed after the given number of cycles into the switch, other faults when
/// the step starts. shutdown must be the last step. Lines starting with # are comments.
namespace cm_test_suite
{
struct FaultSpec
{
  TargetKind kind = TargetKind::Controller;
  std::string name;
  Hook hook = Hook::Update;
  Fault fault = Fault::None;
};

/// Whether the rig can apply the fault. Only async hardware hangs in read or write, a hang of sync
/// hardware there would stop the control loop, which a hold covers.
bool applicable(const FaultSpec & fault);

struct TimedFault
{
  FaultSpec fault;
  int after_cycles = 0;
};

struct SwitchStep
{
  std::vector<std::string> activate;
  std::vector<std::string> deactivate;
  Strictness strictness = Strictness::Strict;
  std::vector<TimedFault> faults;
  /// The control loop stops cycling for longer than the switch timeout.
  bool stall = false;
  /// The control loop is held inside the read of sync hardware until the switch returned.
  bool hold = false;
  /// The power is cut after the given number of cycles into the switch.
  std::optional<int> power_cut;
  std::optional<bool> expect_accepted;
};

/// Armed between operations, so only hooks that fire every cycle are meaningful.
struct FaultStep
{
  FaultSpec fault;
  /// Faults in callbacks that handling the first one calls, such as on_deactivate or on_error.
  std::vector<FaultSpec> handling;
  /// With false the next step starts as soon as the faults fired, while the controller manager
  /// still handles them, and nothing of this step is checked on its own.
  bool wait = true;
};

enum class LifecycleAction
{
  Load,
  Configure,
  Cleanup,
  Unload,
};

struct LifecycleStep
{
  LifecycleAction action = LifecycleAction::Load;
  std::string controller;
  std::optional<FaultSpec> fault;
  std::optional<bool> expect_accepted;
  /// With false the next step starts as soon as this one returned, while its injected hangs may
  /// still run, and nothing of this step is checked on its own.
  bool wait = true;
};

struct RecoverStep
{
  std::string hardware;
  std::optional<FaultSpec> fault;
  std::optional<bool> expect_accepted;
};

/// A hard e-stop cuts the power of the whole rig, as on Hector. Restoring it runs the recovery
/// sequence of hector_controller_spawner: recover every faulted hardware component, then one AUTO
/// switch to the startup controllers, STRICT without the auto_switch capability.
struct PowerStep
{
  bool cut = false;
  /// Wait until the faulted hardware is handled before the next step. Only for a cut.
  bool wait = true;
};

/// Async hardware keeps working but is slow for SLOW_IO_TIME, longer than the 100 ms hang rule.
/// Steadily every read and write takes a millisecond, so one async cycle is slower than the control
/// period. With jitter each takes a random time up to IO_JITTER, with a stall every write takes
/// IO_STALL, with a long stall IO_LONG_STALL. No fault fires.
struct SlowIoStep
{
  SlowIo pace = SlowIo::Steady;
};

/// Shuts the controller manager down, as on process exit.
struct ShutdownStep
{
  std::optional<FaultSpec> fault;
  /// The control loop stops cycling until the shutdown returned.
  bool stall = false;
};

using Step = std::variant<
  SwitchStep, FaultStep, LifecycleStep, RecoverStep, PowerStep, SlowIoStep, ShutdownStep>;
using Script = std::vector<Step>;

std::string to_text(const FaultSpec & fault);
std::string to_text(const Step & step);
std::string to_text(const Script & script);

/// Throws std::invalid_argument with the offending line.
Script parse_script(std::string_view text);

/// A random script using only what the capabilities allow.
Script generate_script(uint64_t seed, size_t steps, const Capabilities & capabilities);

}  // namespace cm_test_suite

#endif  // CM_TEST_SUITE__SCRIPT_HPP_
