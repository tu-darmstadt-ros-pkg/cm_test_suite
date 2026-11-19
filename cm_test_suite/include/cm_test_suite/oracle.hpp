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

#ifndef CM_TEST_SUITE__ORACLE_HPP_
#define CM_TEST_SUITE__ORACLE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "cm_test_suite/cm_under_test.hpp"
#include "cm_test_suite/rig.hpp"
#include "cm_test_suite/script.hpp"
#include "cm_test_suite/trace.hpp"

/// Pure checks of the rig state. They state what every controller manager must guarantee and
/// never predict a design specific outcome, only the bounds every valid outcome lies within.
namespace cm_test_suite::oracle
{
using rig::Names;

struct Snapshot
{
  uint64_t cycle = 0;
  std::vector<ControllerTruth> controllers;
  std::vector<HardwareTruth> hardware;
  /// By interface id.
  std::vector<InterfaceTruth> interfaces;
};

Snapshot take_snapshot(const Trace & trace);
/// Whether the lifecycle related truth of both snapshots is equal, including which interfaces the
/// hardware applies.
bool same_state(const Snapshot & a, const Snapshot & b);

Names controllers_in(const Snapshot & snapshot, Lifecycle state);
Names loaded(const Snapshot & snapshot);
Names hardware_in(const Snapshot & snapshot, Lifecycle state);

/// The controllers a failure of the targets may stop: everything connected to them through
/// controllers that were active before or started during the step, the active unit. A failed
/// controller that was neither active nor started only affects itself.
Names affected_by(
  const std::vector<Target> & failed, const Snapshot & before, const Names & started = {});

/// Controllers a FORCE_AUTO activation of these conflicts with: they claim one of the same command
/// interfaces or command another mode of the same joint.
Names conflicts_of(const Names & controllers);

/// Each returned string describes one violation. Empty means the check passed.
using Findings = std::vector<std::string>;

/// Invariants of a settled system: single claimers, running providers, chained mode, usable
/// hardware, handled faults, no hardware applying an unclaimed interface.
Findings check_consistency(const Snapshot & snapshot);

/// Active controllers that read the state of the hardware without commanding it.
Names state_readers(const Snapshot & snapshot, const std::string & hardware);

/// Hardware active in both snapshots that was not read and written between them, and hardware
/// inactive in both that was not read although an active controller reads its state.
Names hardware_without_io(const Snapshot & begin, const Snapshot & end);

/// Between two settled snapshots active hardware is read and written, every controller active
/// throughout is updated, and every claimed interface whose chain reaches an active writer is
/// applied and receives commands.
Findings check_liveness(const Snapshot & begin, const Snapshot & end);
/// The claimed interfaces check_liveness expects applied that the hardware does not apply.
Names unapplied_claims(const Snapshot & begin, const Snapshot & end);

/// The reported state must equal the truth once the system settled.
Findings check_reported(const Snapshot & snapshot, const ReportedState & reported);

/// A report taken during an operation may only show states the truth passed through, with the
/// same alternatives check_reported accepts. refused_stop names the controllers that refused to
/// stop before or during the operation.
Findings check_reported_during(
  const ReportedState & sample, const std::vector<uint32_t> & visited_controllers,
  const std::vector<uint32_t> & visited_hardware, const Names & refused_stop);

Findings check_startup(const Snapshot & snapshot, const rig::StartupProfile & profile);

struct Outcome
{
  bool accepted;
  const Snapshot & before;
  const Snapshot & after;
  /// Controllers allowed to stop because a fault fired.
  Names excused;
  /// Hardware allowed to change state because a fault fired on it or the step targets it.
  Names hardware_excused;
  /// The step ran until its timeout, so a rollback may have stopped at the deadline.
  bool deadline_passed = false;
  /// Controllers of the unchecked lifecycle steps before this one, which may still change their
  /// state while this step runs, never to or from active.
  Names unchecked_lifecycle{};
};

/// A rejected switch that ran into its deadline may leave the request, the providers claimed by the
/// request and by what AUTO starts for it and under FORCE_AUTO its conflicts stopped, with their
/// active users and the active providers they write, since stopping is safe and a rollback cut
/// short at the deadline restores nothing more.
Findings check_switch(const SwitchStep & step, const Outcome & outcome);
/// A fault may only stop excused controllers and never activates or unloads any.
Findings check_fault(const Outcome & outcome);
/// previous_callback_running: a callback of this controller, left over from an earlier step,
/// still ran when the step started. A load must then be rejected (rulebook 2.2.5).
Findings check_lifecycle(
  const LifecycleStep & step, const Outcome & outcome, bool previous_callback_running = false);
/// A recovery changes no controller, except that one which brings the hardware up from not active
/// or faulted may start inactive controllers of the startup profile.
Findings check_recover(const RecoverStep & step, const Outcome & outcome);
/// Active controllers that command or read hardware which faults without power.
Names running_without_power(const Snapshot & snapshot);
/// Without power no faulting hardware is active and no controller that needs it runs.
Findings check_power_cut(const Snapshot & after);
/// After the recovery sequence every hardware that is not finalized is active again, except where
/// the sequence stopped before recovering it.
Findings check_power_restored(const Snapshot & after, const Names & not_attempted);
/// Nothing may stay active after shutdown, so no drive keeps a command.
Findings check_shutdown(const Snapshot & after);

}  // namespace cm_test_suite::oracle

#endif  // CM_TEST_SUITE__ORACLE_HPP_
