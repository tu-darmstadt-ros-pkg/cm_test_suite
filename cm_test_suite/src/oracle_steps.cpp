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

#include <algorithm>
#include <optional>

#include "cm_test_suite/oracle.hpp"
#include "names.hpp"

namespace cm_test_suite::oracle
{
namespace
{
Names active_unit(const Names & seeds, const Snapshot & before)
{
  std::vector<Target> targets;
  for (const auto & name : seeds)
  {
    targets.push_back({TargetKind::Controller, *rig::controller_index(name)});
  }
  return affected_by(targets, before);
}

/// The active controllers a stop of the seeds takes down with them, their users and the providers
/// they write, as a stopped writer leaves its provider without a reference.
Names stop_closure(const Names & seeds, const Names & active)
{
  const Names active_seeds = intersection(seeds, active);
  const auto stopped_with = [&](std::string_view name)
  { return intersection(merged(rig::users(name), rig::written_providers(name)), active); };
  return merged(active_seeds, rig::closure(active_seeds, stopped_with));
}

/// Freeing a plugin finalized by on_shutdown or a failed on_error may lag into a later step
bool freed(Lifecycle before, Lifecycle after)
{
  return before == Lifecycle::Finalized && after == Lifecycle::Unloaded;
}

/// A late lifecycle step never moves its controller to or from active
bool late_change(
  const Outcome & outcome, const std::string & name, Lifecycle before, Lifecycle after)
{
  return outcome.unchecked_lifecycle.count(name) > 0 && before != Lifecycle::Active &&
         after != Lifecycle::Active;
}

void expect_controllers_unchanged(
  const Outcome & outcome, const std::string & except, Findings & findings,
  const Names & may_start = {})
{
  for (const auto & c : rig::controllers())
  {
    const auto before = controller_state(outcome.before, c.name);
    const auto after = controller_state(outcome.after, c.name);
    // A fired fault may stop its active unit, as in any other step
    const bool stopped_by_fault = outcome.excused.count(c.name) > 0 && after != Lifecycle::Active;
    const bool started =
      may_start.count(c.name) > 0 && before == Lifecycle::Inactive && after == Lifecycle::Active;
    if (
      c.name != except && before != after && !stopped_by_fault && !started &&
      !late_change(outcome, c.name, before, after) && !freed(before, after))
    {
      findings.push_back(
        c.name + " changed from " + to_string(before) + " to " + to_string(after) +
        " although the step did not concern it");
    }
  }
}

void expect_hardware_unchanged(const Outcome & outcome, Findings & findings)
{
  for (const auto & hw : rig::hardware())
  {
    const auto before = hardware_state(outcome.before, hw.name);
    const auto after = hardware_state(outcome.after, hw.name);
    if (before != after && !outcome.hardware_excused.count(hw.name))
    {
      findings.push_back(
        "hardware " + hw.name + " changed from " + to_string(before) + " to " + to_string(after) +
        " although no fault hit it");
    }
  }
}

void expect_answer(const std::optional<bool> & expected, bool accepted, Findings & findings)
{
  if (expected && *expected != accepted)
  {
    findings.push_back(
      accepted ? "accepted, but must be rejected" : "rejected, but must be accepted");
  }
}
}  // namespace

Names conflicts_of(const Names & controllers)
{
  Names result;
  for (const auto & name : controllers)
  {
    result.merge(rig::command_conflicts(name));
  }
  return without(result, controllers);
}

Findings check_switch(const SwitchStep & step, const Outcome & outcome)
{
  Findings findings;
  expect_answer(step.expect_accepted, outcome.accepted, findings);
  expect_hardware_unchanged(outcome, findings);
  const Names activate = from(step.activate);
  const Names deactivate = from(step.deactivate);
  const Names before = controllers_in(outcome.before, Lifecycle::Active);
  const Names after = controllers_in(outcome.after, Lifecycle::Active);
  const Names gained = without(after, before);
  const Names lost = without(before, after);

  const bool expands = step.strictness != Strictness::Strict;
  const Names may_gain =
    expands ? merged(activate, rig::closure(activate, rig::providers)) : activate;
  Names may_lose = merged(deactivate, outcome.excused);
  if (step.strictness == Strictness::ForceAuto)
  {
    may_lose.merge(active_unit(merged(conflicts_of(may_gain), deactivate), outcome.before));
  }

  if (outcome.accepted)
  {
    const Names not_running = without(without(activate, after), outcome.excused);
    if (!not_running.empty())
    {
      findings.push_back("accepted, but requested " + text(not_running) + " are not active");
    }
    const Names still_running = intersection(without(deactivate, activate), after);
    if (!still_running.empty())
    {
      findings.push_back("accepted, but " + text(still_running) + " are still active");
    }
    for (const auto & name : intersection(intersection(activate, deactivate), after))
    {
      if (
        before.count(name) &&
        controller_truth(outcome.before, name).epoch == controller_truth(outcome.after, name).epoch)
      {
        findings.push_back("accepted, but " + name + " was not restarted");
      }
    }
  }
  else if (!gained.empty())
  {
    findings.push_back("rejected, but activated " + text(gained));
  }
  if (!is_subset(gained, may_gain))
  {
    findings.push_back("activated " + text(without(gained, may_gain)) + " beyond the request");
  }
  // Nothing is reactivated because of a fault (rulebook 2.2.4), so a rejection after a fault fired
  // may leave stopped what the request itself may stop
  const bool fault_fired = !outcome.excused.empty();
  Names may_lose_now = outcome.accepted || fault_fired ? may_lose : outcome.excused;
  if (!outcome.accepted && outcome.deadline_passed)
  {
    // Restarting the request or what AUTO starts stops the providers they claim and what that stops
    const Names request = merged(merged(activate, deactivate), without(may_gain, before));
    Names stops = merged(request, rig::closure(request, rig::written_providers));
    if (step.strictness == Strictness::ForceAuto)
    {
      stops.merge(conflicts_of(may_gain));
    }
    may_lose_now.merge(stop_closure(stops, before));
  }
  if (!is_subset(lost, may_lose_now))
  {
    findings.push_back(
      std::string(outcome.accepted ? "accepted" : "rejected") + ", but deactivated " +
      text(without(lost, may_lose_now)) + " beyond the request");
  }
  return findings;
}

Findings check_fault(const Outcome & outcome)
{
  Findings findings;
  expect_hardware_unchanged(outcome, findings);
  const Names before = controllers_in(outcome.before, Lifecycle::Active);
  const Names after = controllers_in(outcome.after, Lifecycle::Active);
  const Names gained = without(after, before);
  const Names lost = without(without(before, after), outcome.excused);
  if (!gained.empty())
  {
    findings.push_back("a fault activated " + text(gained));
  }
  if (!lost.empty())
  {
    findings.push_back("a fault deactivated unaffected " + text(lost));
  }
  const Names unloaded = without(
    without(without(loaded(outcome.before), loaded(outcome.after)), outcome.unchecked_lifecycle),
    controllers_in(outcome.before, Lifecycle::Finalized));
  if (!unloaded.empty())
  {
    findings.push_back("a fault unloaded " + text(unloaded));
  }
  return findings;
}

Findings check_lifecycle(
  const LifecycleStep & step, const Outcome & outcome, bool previous_callback_running)
{
  Findings findings;
  expect_answer(step.expect_accepted, outcome.accepted, findings);
  expect_hardware_unchanged(outcome, findings);
  expect_controllers_unchanged(outcome, step.controller, findings);
  const auto before = controller_state(outcome.before, step.controller);
  const auto after = controller_state(outcome.after, step.controller);
  bool allowed = false;
  std::vector<Lifecycle> expected;
  switch (step.action)
  {
    case LifecycleAction::Load:
      // A finalized plugin may already be freed and reported unloaded (check_reported)
      allowed = before == Lifecycle::Unloaded || before == Lifecycle::Finalized;
      expected = {Lifecycle::Unconfigured};
      break;
    case LifecycleAction::Configure:
      allowed = before == Lifecycle::Unconfigured || before == Lifecycle::Inactive;
      expected = {Lifecycle::Inactive};
      break;
    case LifecycleAction::Cleanup:
      allowed = before == Lifecycle::Inactive || before == Lifecycle::Unconfigured;
      expected = {Lifecycle::Unconfigured};
      break;
    case LifecycleAction::Unload:
      allowed = before != Lifecycle::Unloaded && before != Lifecycle::Active;
      expected = {Lifecycle::Unloaded, Lifecycle::Finalized};
      break;
  }
  if (outcome.accepted && !allowed)
  {
    findings.push_back("accepted from state " + std::string(to_string(before)));
  }
  const bool reached = std::find(expected.begin(), expected.end(), after) != expected.end();
  if (outcome.accepted && !reached)
  {
    findings.push_back("accepted, but " + step.controller + " is " + to_string(after));
  }
  // A failing callback may leave an error state and a timed out unload may still finish
  const bool fault_fired = outcome.excused.count(step.controller) > 0;
  const bool error_state = after == Lifecycle::Unconfigured || after == Lifecycle::Finalized ||
                           (step.action == LifecycleAction::Unload && after == Lifecycle::Unloaded);
  if (outcome.accepted && step.action == LifecycleAction::Load && previous_callback_running)
  {
    findings.push_back(
      "accepted while a callback of the previous " + step.controller + " still ran");
  }
  // The earlier operation whose callback still runs may change the state meanwhile
  if (
    !outcome.accepted && after != before && !(fault_fired && error_state) &&
    !previous_callback_running && !late_change(outcome, step.controller, before, after) &&
    !freed(before, after))
  {
    findings.push_back(
      "rejected, but " + step.controller + " changed from " + to_string(before) + " to " +
      to_string(after));
  }
  return findings;
}

Findings check_recover(const RecoverStep & step, const Outcome & outcome)
{
  Findings findings;
  expect_answer(step.expect_accepted, outcome.accepted, findings);
  expect_hardware_unchanged(outcome, findings);
  // An accepted recovery that brings the hardware up may run the startup switch, the rig has no
  // e-stop that could hold it back. Restarting what an operator stopped on hardware that was up
  // is a bug.
  const auto & before = hardware_truth(outcome.before, step.hardware);
  const bool brought_up = before.state != Lifecycle::Active || before.faulted;
  expect_controllers_unchanged(
    outcome, "", findings,
    outcome.accepted && brought_up ? rig::startup_profile().controllers_active : Names{});
  const auto after = hardware_state(outcome.after, step.hardware);
  if (outcome.accepted && after != Lifecycle::Active)
  {
    findings.push_back("accepted, but " + step.hardware + " is " + to_string(after));
  }
  return findings;
}

}  // namespace cm_test_suite::oracle
