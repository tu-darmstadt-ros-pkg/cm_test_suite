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

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "cm_test_suite/oracle.hpp"

namespace cm_test_suite::oracle
{
namespace
{
/// A settled state: every controller loaded and inactive except the active ones, the hardware
/// active except the down ones, and the hardware applying exactly the claimed interfaces.
Snapshot snapshot(const Names & active, const Names & down = {})
{
  Snapshot s;
  s.cycle = 100;
  for (const auto & c : rig::controllers())
  {
    const bool on = active.count(c.name) > 0;
    s.controllers.push_back(
      {on ? Lifecycle::Active : Lifecycle::Inactive, 1, false, 0, false, 100});
  }
  for (const auto & hw : rig::hardware())
  {
    const bool off = down.count(hw.name) > 0;
    s.hardware.push_back({off ? Lifecycle::Unconfigured : Lifecycle::Active, false, 100, 100});
  }
  s.interfaces.resize(rig::interface_count(), {false, 0});
  for (const auto & c : rig::controllers())
  {
    const Names writers = rig::reference_writers(c.name);
    const bool written = std::any_of(
      writers.begin(), writers.end(), [&](const std::string & w) { return active.count(w) > 0; });
    s.controllers[*rig::controller_index(c.name)].chained = written;
    for (const auto & name : c.command_interfaces)
    {
      const auto hw = rig::hardware_of_interface(name);
      if (active.count(c.name) && hw && !down.count(*hw))
      {
        s.interfaces[*rig::interface_id(name)] = {true, 100};
      }
    }
  }
  return s;
}

/// The same state some cycles later, with controllers updated, hardware read and written and
/// commands applied.
Snapshot later(Snapshot s)
{
  s.cycle += 10;
  for (auto & controller : s.controllers)
  {
    controller.updates += 10;
  }
  for (auto & hw : s.hardware)
  {
    hw.reads += 10;
    hw.writes += 10;
  }
  for (auto & interface : s.interfaces)
  {
    if (interface.live)
    {
      interface.applied_cycle = s.cycle;
    }
  }
  return s;
}

ControllerTruth & truth(Snapshot & s, const std::string & name)
{
  return s.controllers[*rig::controller_index(name)];
}

HardwareTruth & truth_hw(Snapshot & s, const std::string & name)
{
  return s.hardware[*rig::hardware_index(name)];
}

const Names STARTUP = rig::startup_profile().controllers_active;

SwitchStep request(
  Strictness strictness, std::vector<std::string> activate,
  std::vector<std::string> deactivate = {})
{
  SwitchStep step;
  step.activate = std::move(activate);
  step.deactivate = std::move(deactivate);
  step.strictness = strictness;
  return step;
}

Outcome outcome(bool accepted, const Snapshot & before, const Snapshot & after, Names excused = {})
{
  return {accepted, before, after, std::move(excused), {}};
}
}  // namespace

TEST(Settle, a_started_or_held_interface_is_a_change)
{
  const Snapshot before = snapshot(STARTUP);
  Snapshot after = later(before);
  EXPECT_TRUE(same_state(before, after));
  after.interfaces[*rig::interface_id("a1/velocity")].live = false;
  EXPECT_FALSE(same_state(before, after));
}

TEST(Consistency, accepts_the_startup_profile)
{
  EXPECT_TRUE(check_consistency(snapshot(STARTUP)).empty());
}

TEST(Consistency, finds_a_user_above_an_inactive_provider)
{
  EXPECT_FALSE(check_consistency(snapshot({"teleop"})).empty());
}

TEST(Consistency, finds_a_provider_chained_without_writer)
{
  Snapshot s = snapshot({"arm_vel"});
  truth(s, "arm_vel").chained = true;
  EXPECT_EQ(1u, check_consistency(s).size());
}

TEST(Consistency, finds_a_provider_unchained_under_an_active_writer)
{
  Snapshot s = snapshot({"arm_vel", "wrist"});
  truth(s, "arm_vel").chained = false;
  EXPECT_EQ(1u, check_consistency(s).size());
}

TEST(Consistency, finds_shared_command_interfaces)
{
  Names active = STARTUP;
  active.insert({"whole_body", "nav"});
  EXPECT_FALSE(check_consistency(snapshot(active)).empty());
}

TEST(Consistency, finds_controllers_on_unavailable_hardware)
{
  EXPECT_FALSE(check_consistency(snapshot({"flipper_vel"}, {"flipper"})).empty());
  Snapshot readable = snapshot({"odom_pub", "diff_drive", "base_vel"});
  readable.hardware[*rig::hardware_index("imu")].state = Lifecycle::Inactive;
  EXPECT_TRUE(check_consistency(readable).empty());
}

TEST(Consistency, best_effort_readers_may_skip_down_hardware)
{
  EXPECT_TRUE(check_consistency(snapshot({"jsb"}, {"flipper"})).empty());
}

TEST(Consistency, finds_ignored_faults)
{
  Snapshot hardware = snapshot(STARTUP);
  hardware.hardware[*rig::hardware_index("arm")].faulted = true;
  EXPECT_EQ(1u, check_consistency(hardware).size());
  Snapshot controller = snapshot(STARTUP);
  truth(controller, "arm_ik").failed_epoch = 1;
  EXPECT_EQ(1u, check_consistency(controller).size());
}

TEST(Consistency, hardware_applies_exactly_the_claimed_interfaces)
{
  Snapshot unclaimed = snapshot({});
  unclaimed.interfaces[*rig::interface_id("f1/position")].live = true;
  EXPECT_EQ(1u, check_consistency(unclaimed).size());
  Snapshot not_started = snapshot({"flipper_vel"});
  not_started.interfaces[*rig::interface_id("f1/velocity")].live = false;
  EXPECT_TRUE(check_consistency(not_started).empty());
  EXPECT_EQ(1u, check_liveness(not_started, later(not_started)).size());
}

TEST(Liveness, requires_io_and_commands_while_settling)
{
  const Snapshot begin = snapshot(STARTUP);
  EXPECT_TRUE(check_liveness(begin, later(begin)).empty());
  Snapshot no_writes = later(begin);
  no_writes.hardware[*rig::hardware_index("base")].writes = begin.hardware[1].writes;
  EXPECT_EQ(1u, check_liveness(begin, no_writes).size());
  Snapshot no_commands = later(begin);
  no_commands.interfaces[*rig::interface_id("f1/position")].applied_cycle = begin.cycle;
  EXPECT_EQ(1u, check_liveness(begin, no_commands).size());
}

TEST(Liveness, inactive_hardware_is_read_while_a_controller_reads_it)
{
  Snapshot begin = snapshot({"jsb"});
  truth_hw(begin, "flipper").state = Lifecycle::Inactive;
  Snapshot end = later(begin);
  truth_hw(end, "flipper").reads = truth_hw(begin, "flipper").reads;
  truth_hw(end, "flipper").writes = truth_hw(begin, "flipper").writes;
  EXPECT_FALSE(check_liveness(begin, end).empty());
  truth_hw(end, "flipper").reads += 10;
  EXPECT_TRUE(check_liveness(begin, end).empty());
  Snapshot unread_without_reader = snapshot({});
  truth_hw(unread_without_reader, "flipper").state = Lifecycle::Inactive;
  Snapshot later_without_reader = later(unread_without_reader);
  truth_hw(later_without_reader, "flipper").reads =
    truth_hw(unread_without_reader, "flipper").reads;
  EXPECT_TRUE(check_liveness(unread_without_reader, later_without_reader).empty());
}

TEST(Liveness, every_controller_active_throughout_is_updated)
{
  Snapshot begin = snapshot({"monitor"});
  Snapshot paused = later(begin);
  truth(paused, "monitor").updates = truth(begin, "monitor").updates;
  EXPECT_EQ(1u, check_liveness(begin, paused).size());
  Snapshot restarted = paused;
  ++truth(restarted, "monitor").epoch;
  EXPECT_TRUE(check_liveness(begin, restarted).empty()) << "only a whole activation must run";
  Snapshot stopped = paused;
  truth(stopped, "monitor").state = Lifecycle::Inactive;
  EXPECT_TRUE(check_liveness(begin, stopped).empty());
  Snapshot inactive = snapshot({});
  Snapshot started = later(inactive);
  truth(started, "monitor").state = Lifecycle::Active;
  truth(started, "monitor").updates = truth(inactive, "monitor").updates;
  EXPECT_TRUE(check_liveness(inactive, started).empty());
}

TEST(Switch, strict_must_not_add_providers)
{
  const Snapshot before = snapshot({});
  const Snapshot after = snapshot({"arm_vel", "wrist"});
  EXPECT_FALSE(
    check_switch(request(Strictness::Strict, {"wrist"}), outcome(true, before, after)).empty());
  EXPECT_TRUE(
    check_switch(request(Strictness::Auto, {"wrist"}), outcome(true, before, after)).empty());
}

TEST(Switch, auto_never_stops_what_was_not_requested)
{
  const Snapshot before = snapshot({"base_vel", "diff_drive", "odom_pub"});
  const Snapshot after = snapshot({});
  EXPECT_FALSE(
    check_switch(request(Strictness::Auto, {}, {"base_vel"}), outcome(true, before, after))
      .empty());
  EXPECT_TRUE(
    check_switch(request(Strictness::ForceAuto, {}, {"base_vel"}), outcome(true, before, after))
      .empty());
}

TEST(Switch, rejection_changes_nothing_unless_excused)
{
  const Snapshot before = snapshot({"arm_vel", "wrist"});
  const Snapshot after = snapshot({"arm_vel"});
  const auto step = request(Strictness::Strict, {"arm_ik"});
  EXPECT_FALSE(check_switch(step, outcome(false, before, after)).empty());
  EXPECT_TRUE(check_switch(step, outcome(false, before, after, {"wrist"})).empty());
}

TEST(Switch, force_auto_may_stop_the_conflicting_unit_only)
{
  const Snapshot before = snapshot({"base_vel", "diff_drive", "nav", "odom_pub", "jsb", "arm_vel"});
  const Snapshot after = snapshot(
    {"base_vel", "diff_drive", "whole_body", "arm_vel", "arm_ik", "wrist", "flipper_pos",
     "teleop"});
  const auto findings =
    check_switch(request(Strictness::ForceAuto, {"teleop"}), outcome(true, before, after));
  ASSERT_EQ(1u, findings.size()) << findings.front();
  EXPECT_NE(std::string::npos, findings.front().find("[jsb]"));
}

TEST(Switch, a_rejection_after_a_fault_may_leave_the_request_stopped)
{
  const Snapshot before = snapshot(STARTUP);
  const Snapshot after = snapshot({"jsb", "base_vel", "diff_drive", "flipper_pos", "odom_pub"});
  const auto step = request(Strictness::ForceAuto, {"arm_pos"}, {"wrist"});
  EXPECT_TRUE(check_switch(step, outcome(false, before, after, {"arm_pos"})).empty());
  EXPECT_FALSE(check_switch(step, outcome(false, before, after)).empty());
  const Snapshot unrelated = snapshot({"base_vel", "diff_drive", "flipper_pos", "odom_pub"});
  EXPECT_FALSE(check_switch(step, outcome(false, before, unrelated, {"arm_pos"})).empty());
}

TEST(Switch, a_rejection_at_the_deadline_may_leave_what_a_stop_of_the_request_reaches_stopped)
{
  // switch strict -wrist hold, whose rollback the deadline cut short
  const Snapshot before = snapshot(STARTUP);
  const Snapshot after = snapshot({"jsb", "base_vel", "diff_drive", "flipper_pos", "odom_pub"});
  const auto step = request(Strictness::Strict, {}, {"wrist"});
  Outcome at_deadline = outcome(false, before, after);
  at_deadline.deadline_passed = true;
  EXPECT_TRUE(check_switch(step, at_deadline).empty());
  EXPECT_FALSE(check_switch(step, outcome(false, before, after)).empty())
    << "a rejection in time restores the unit";
  // Outcome keeps references, so every snapshot needs a name
  const Snapshot without_diff_drive =
    snapshot({"jsb", "arm_vel", "arm_ik", "base_vel", "flipper_pos", "odom_pub"});
  Outcome unrelated = outcome(false, before, without_diff_drive);
  unrelated.deadline_passed = true;
  EXPECT_FALSE(check_switch(step, unrelated).empty()) << "diff_drive is outside the unit";
  // A stop of wrist stops its user whole_body and so the providers whole_body writes
  Names with_whole_body = STARTUP;
  with_whole_body.insert("whole_body");
  const Snapshot whole_body_before = snapshot(with_whole_body);
  const auto restart = request(Strictness::Strict, {"wrist"}, {"wrist"});
  const Snapshot only_jsb = snapshot({"jsb"});
  Outcome shared_provider = outcome(false, whole_body_before, only_jsb);
  shared_provider.deadline_passed = true;
  EXPECT_TRUE(check_switch(restart, shared_provider).empty());
  const Snapshot all_lost = snapshot({});
  Outcome jsb_lost = outcome(false, whole_body_before, all_lost);
  jsb_lost.deadline_passed = true;
  const auto findings = check_switch(restart, jsb_lost);
  ASSERT_EQ(1u, findings.size());
  EXPECT_NE(std::string::npos, findings.front().find("deactivated [jsb]")) << findings.front();
  Names more = STARTUP;
  more.insert("monitor");
  const Snapshot with_monitor = snapshot(more);
  Outcome gained = outcome(false, before, with_monitor);
  gained.deadline_passed = true;
  EXPECT_FALSE(check_switch(step, gained).empty()) << "a rejection never activates anything";
  // odom_pub only reads the state diff_drive exports, so a stop of it leaves diff_drive running
  const Snapshot without_odom_unit = snapshot({"jsb", "arm_vel", "arm_ik", "wrist", "flipper_pos"});
  Outcome state_provider = outcome(false, before, without_odom_unit);
  state_provider.deadline_passed = true;
  EXPECT_FALSE(
    check_switch(request(Strictness::Strict, {"odom_pub"}, {"odom_pub"}), state_provider).empty());
}

TEST(Switch, a_rejection_at_the_deadline_may_leave_what_auto_started_providers_write_stopped)
{
  // switch auto +odom_pub hold starts diff_drive, which restarts base_vel into chained mode
  const Snapshot before =
    snapshot({"jsb", "arm_vel", "arm_ik", "wrist", "base_vel", "flipper_pos"});
  const Snapshot after = snapshot({"jsb", "arm_vel", "arm_ik", "wrist", "flipper_pos"});
  Outcome at_deadline = outcome(false, before, after);
  at_deadline.deadline_passed = true;
  EXPECT_TRUE(check_switch(request(Strictness::Auto, {"odom_pub"}), at_deadline).empty());
  EXPECT_TRUE(check_switch(request(Strictness::ForceAuto, {"odom_pub"}), at_deadline).empty());
  EXPECT_FALSE(check_switch(request(Strictness::Strict, {"odom_pub"}), at_deadline).empty())
    << "STRICT starts no provider";
  // With diff_drive already running AUTO starts nothing that writes base_vel
  Names without_odom_pub = STARTUP;
  without_odom_pub.erase("odom_pub");
  const Snapshot running = snapshot(without_odom_pub);
  Outcome lost = outcome(false, running, after);
  lost.deadline_passed = true;
  EXPECT_FALSE(check_switch(request(Strictness::Auto, {"odom_pub"}), lost).empty());
}

TEST(Switch, a_force_auto_rejection_at_the_deadline_may_leave_its_conflicts_stopped)
{
  // switch force_auto +flipper_vel hold, whose rollback the deadline cut short
  const Snapshot before = snapshot(STARTUP);
  Names without_flipper_pos = STARTUP;
  without_flipper_pos.erase("flipper_pos");
  const Snapshot after = snapshot(without_flipper_pos);
  Outcome at_deadline = outcome(false, before, after);
  at_deadline.deadline_passed = true;
  EXPECT_TRUE(check_switch(request(Strictness::ForceAuto, {"flipper_vel"}), at_deadline).empty());
  EXPECT_FALSE(check_switch(request(Strictness::Strict, {"flipper_vel"}), at_deadline).empty())
    << "only FORCE_AUTO stops conflicts";
  EXPECT_FALSE(check_switch(request(Strictness::ForceAuto, {"nav"}), at_deadline).empty())
    << "a stop of the inactive conflict whole_body leaves flipper_pos running";
  without_flipper_pos.erase("arm_vel");
  const Snapshot arm_lost = snapshot(without_flipper_pos);
  Outcome unrelated = outcome(false, before, arm_lost);
  unrelated.deadline_passed = true;
  EXPECT_FALSE(check_switch(request(Strictness::ForceAuto, {"flipper_vel"}), unrelated).empty())
    << "arm_vel does not conflict with flipper_vel";
}

TEST(Switch, force_auto_stops_nothing_for_an_inactive_conflict)
{
  Names with_nav = STARTUP;
  with_nav.insert("nav");
  with_nav.erase("flipper_pos");
  const Snapshot before = snapshot(STARTUP);
  const Snapshot after = snapshot(with_nav);
  const Outcome flipper_lost = outcome(true, before, after);
  EXPECT_FALSE(check_switch(request(Strictness::ForceAuto, {"nav"}), flipper_lost).empty())
    << "the conflict whole_body is inactive";
  EXPECT_FALSE(
    check_switch(request(Strictness::ForceAuto, {"nav"}, {"whole_body"}), flipper_lost).empty())
    << "a deactivation of the inactive whole_body stops nothing";
}

TEST(Recover, a_fired_fault_may_stop_its_active_unit)
{
  const Snapshot before = snapshot(STARTUP);
  Snapshot after = snapshot({"jsb", "arm_vel", "arm_ik", "wrist", "flipper_pos"});
  const RecoverStep step{"imu", std::nullopt, std::nullopt};
  const Names unit = {"base_vel", "diff_drive", "odom_pub"};
  Outcome excused{false, before, after, unit, {"imu"}};
  EXPECT_TRUE(check_recover(step, excused).empty());
  Outcome unexcused{false, before, after, {}, {"imu"}};
  EXPECT_FALSE(check_recover(step, unexcused).empty());
  Names more = STARTUP;
  more.insert("nav");
  Outcome gained{false, before, snapshot(more), unit, {"imu"}};
  EXPECT_FALSE(check_recover(step, gained).empty());
}

// The startup switch after a recovery may bring back the startup set
TEST(Recover, may_start_only_startup_controllers)
{
  const RecoverStep step{"imu", std::nullopt, std::nullopt};
  const Snapshot before = snapshot({"jsb", "flipper_pos"}, {"imu"});
  EXPECT_TRUE(check_recover(step, {true, before, snapshot(STARTUP), {}, {"imu"}}).empty());
  Names more = STARTUP;
  more.insert("nav");
  EXPECT_FALSE(check_recover(step, {true, before, snapshot(more), {}, {"imu"}}).empty());
  // Whether its hardware allows a start is for the consistency check and the events to judge
  const Snapshot unpowered = snapshot({"jsb", "flipper_pos"}, {"imu", "base"});
  const Snapshot base_vel_on_unpowered = snapshot({"jsb", "flipper_pos", "base_vel"}, {"base"});
  EXPECT_TRUE(check_recover(step, {true, unpowered, base_vel_on_unpowered, {}, {"imu"}}).empty());
  // diff_drive and odom_pub only read the imu, which may stay inactive
  Snapshot imu_inactive = snapshot(STARTUP);
  imu_inactive.hardware[*rig::hardware_index("imu")].state = Lifecycle::Inactive;
  const RecoverStep arm{"arm", std::nullopt, std::nullopt};
  const Snapshot arm_down = snapshot({"jsb", "flipper_pos"}, {"arm", "imu"});
  EXPECT_TRUE(check_recover(arm, {true, arm_down, imu_inactive, {}, {"arm", "imu"}}).empty());
  EXPECT_FALSE(
    check_recover(step, {true, snapshot(STARTUP), snapshot({"jsb"}), {}, {"imu"}}).empty())
    << "a recovery stops nothing";
  const Snapshot base_vel_back = snapshot({"jsb", "flipper_pos", "base_vel"}, {"imu"});
  EXPECT_FALSE(check_recover(step, {false, before, base_vel_back, {}, {"imu"}}).empty())
    << "a rejected recovery starts nothing";
}

// Only an activation that brings the hardware up runs the startup switch
TEST(Recover, starts_nothing_on_hardware_that_was_up)
{
  const RecoverStep step{"imu", std::nullopt, std::nullopt};
  const Snapshot up = snapshot({"jsb", "flipper_pos"});
  EXPECT_FALSE(check_recover(step, {true, up, snapshot(STARTUP), {}, {"imu"}}).empty());
  Snapshot faulted = up;
  faulted.hardware[*rig::hardware_index("imu")].faulted = true;
  EXPECT_TRUE(check_recover(step, {true, faulted, snapshot(STARTUP), {}, {"imu"}}).empty())
    << "a fault the status did not show yet counts as down";
}

TEST(Switch, force_auto_must_not_stop_unconnected_chains)
{
  // The arm chain and the base chain are only connected through whole_body, which is inactive
  const Snapshot before = snapshot({"arm_vel", "base_vel", "diff_drive", "nav"});
  const Names wanted = {"whole_body", "arm_ik",   "wrist",      "arm_vel",
                        "diff_drive", "base_vel", "flipper_pos"};
  const auto step = request(Strictness::ForceAuto, {"whole_body"});
  EXPECT_TRUE(check_switch(step, outcome(true, before, snapshot(wanted))).empty());
  Snapshot restarted_arm = snapshot(wanted);
  truth(restarted_arm, "arm_vel").epoch = 2;
  EXPECT_TRUE(check_switch(step, outcome(true, before, restarted_arm)).empty());
  Names without_arm = wanted;
  without_arm.erase("arm_vel");
  const auto findings = check_switch(step, outcome(true, before, snapshot(without_arm)));
  ASSERT_FALSE(findings.empty());
  EXPECT_NE(std::string::npos, findings.back().find("[arm_vel]"));
}

TEST(Switch, accepted_requests_must_take_effect)
{
  const Snapshot before = snapshot({"jsb"});
  EXPECT_FALSE(
    check_switch(request(Strictness::Strict, {"monitor"}, {"jsb"}), outcome(true, before, before))
      .empty());
}

TEST(Switch, a_restart_must_change_the_epoch)
{
  const Snapshot before = snapshot({"monitor"});
  Snapshot after = before;
  const auto step = request(Strictness::Strict, {"monitor"}, {"monitor"});
  EXPECT_FALSE(check_switch(step, outcome(true, before, after)).empty());
  truth(after, "monitor").epoch = 2;
  EXPECT_TRUE(check_switch(step, outcome(true, before, after)).empty());
}

TEST(Switch, hardware_may_only_change_where_a_fault_hit)
{
  const Snapshot before = snapshot({"monitor"});
  const Snapshot after = snapshot({"monitor"}, {"arm"});
  EXPECT_FALSE(
    check_switch(request(Strictness::Strict, {"jsb"}), outcome(false, before, after)).empty());
  Outcome excused{false, before, after, {}, {"arm"}};
  EXPECT_TRUE(check_switch(request(Strictness::Strict, {"jsb"}), excused).empty());
}

TEST(Fault, never_unloads_a_controller)
{
  const Snapshot before = snapshot(STARTUP);
  Snapshot after = snapshot(STARTUP);
  EXPECT_TRUE(check_fault(outcome(true, before, after)).empty());
  truth(after, "nav").state = Lifecycle::Unloaded;
  EXPECT_FALSE(check_fault(outcome(true, before, after)).empty());
}

TEST(Fault, may_free_a_plugin_finalized_before)
{
  // A CM frees a plugin off RT after its on_shutdown returned, which can be after settling
  Snapshot before = snapshot({});
  truth(before, "monitor").state = Lifecycle::Finalized;
  Snapshot after = snapshot({});
  truth(after, "monitor").state = Lifecycle::Unloaded;
  EXPECT_TRUE(check_fault(outcome(true, before, after)).empty());
  const LifecycleStep configure{LifecycleAction::Configure, "nav", std::nullopt, std::nullopt};
  EXPECT_TRUE(check_lifecycle(configure, outcome(true, before, after)).empty());
  const LifecycleStep cleanup{LifecycleAction::Cleanup, "monitor", std::nullopt, std::nullopt};
  EXPECT_TRUE(check_lifecycle(cleanup, outcome(false, before, after)).empty());
}

TEST(Fault, never_activates_and_stops_only_excused_controllers)
{
  const Snapshot before = snapshot(STARTUP);
  Names more = STARTUP;
  more.insert("monitor");
  const Snapshot started = snapshot(more);
  EXPECT_EQ(1u, check_fault(outcome(true, before, started)).size());
  Names fewer = STARTUP;
  fewer.erase("odom_pub");
  const Snapshot stopped = snapshot(fewer);
  EXPECT_EQ(1u, check_fault(outcome(true, before, stopped)).size());
  EXPECT_TRUE(check_fault(outcome(true, before, stopped, {"odom_pub"})).empty());
}

TEST(Switch, a_rejection_activates_nothing_even_within_the_request)
{
  const Snapshot before = snapshot({});
  const Snapshot after = snapshot({"monitor"});
  EXPECT_EQ(
    1u,
    check_switch(request(Strictness::Strict, {"monitor"}), outcome(false, before, after)).size());
}

TEST(Switch, expectations_are_enforced)
{
  const Snapshot s = snapshot({});
  auto step = request(Strictness::Strict, {"monitor"});
  step.expect_accepted = true;
  EXPECT_FALSE(check_switch(step, outcome(false, s, s)).empty());
}

TEST(AffectedBy, stays_within_the_active_unit)
{
  const Snapshot before = snapshot({"arm_vel", "wrist", "base_vel", "diff_drive", "odom_pub"});
  const Names unit =
    affected_by({{TargetKind::Controller, *rig::controller_index("wrist")}}, before);
  EXPECT_EQ(Names({"arm_vel", "wrist"}), unit);
  EXPECT_EQ(
    Names({"jsb"}), affected_by({{TargetKind::Controller, *rig::controller_index("jsb")}}, before));
  const Names imu = affected_by({{TargetKind::Hardware, *rig::hardware_index("imu")}}, before);
  EXPECT_EQ(Names({"base_vel", "diff_drive", "odom_pub"}), imu);
}

TEST(AffectedBy, includes_units_formed_during_the_step)
{
  const Snapshot before = snapshot({"arm_vel", "base_vel", "diff_drive"});
  const Target base{TargetKind::Hardware, *rig::hardware_index("base")};
  EXPECT_FALSE(affected_by({base}, before).count("arm_vel"));
  EXPECT_TRUE(affected_by({base}, before, {"whole_body", "arm_ik"}).count("arm_vel"));
}

TEST(AffectedBy, a_fault_in_an_inactive_controller_stops_nothing_around_it)
{
  const Snapshot before = snapshot(STARTUP);
  const Target whole_body{TargetKind::Controller, *rig::controller_index("whole_body")};
  EXPECT_EQ(Names({"whole_body"}), affected_by({whole_body}, before));
  EXPECT_TRUE(affected_by({whole_body}, before, {"whole_body", "arm_ik"}).count("arm_vel"))
    << "the step started whole_body";
}

TEST(Conflicts, include_other_modes_of_the_same_joint)
{
  EXPECT_EQ(Names({"flipper_pos"}), conflicts_of({"flipper_vel"}));
  EXPECT_EQ(Names({"arm_vel"}), conflicts_of({"arm_pos"}));
  EXPECT_EQ(Names({"nav"}), conflicts_of({"whole_body"}));
}

TEST(Liveness, a_provider_without_writer_may_hold_or_write_nothing)
{
  // arm_ik stopped, so arm_vel's a1 and a2 have no writer, while wrist still writes a3
  Snapshot held = snapshot({"arm_vel", "wrist"});
  for (const auto * name : {"a1/velocity", "a2/velocity"})
  {
    held.interfaces[*rig::interface_id(name)].live = false;
  }
  EXPECT_TRUE(check_liveness(held, later(held)).empty());
  const Snapshot no_commands = snapshot({"arm_vel", "wrist"});
  Snapshot next = later(no_commands);
  for (const auto * name : {"a1/velocity", "a2/velocity"})
  {
    next.interfaces[*rig::interface_id(name)].applied_cycle = 0;
  }
  EXPECT_TRUE(check_liveness(no_commands, next).empty());
  next.interfaces[*rig::interface_id("a3/velocity")].applied_cycle = 0;
  EXPECT_EQ(1u, check_liveness(no_commands, next).size());
}

TEST(Lifecycle, active_controllers_cannot_be_unloaded)
{
  const Snapshot before = snapshot({"monitor"});
  Snapshot after = before;
  truth(after, "monitor").state = Lifecycle::Unloaded;
  const LifecycleStep unload{LifecycleAction::Unload, "monitor", std::nullopt, std::nullopt};
  EXPECT_FALSE(check_lifecycle(unload, outcome(true, before, after)).empty());
  EXPECT_TRUE(check_lifecycle(unload, outcome(false, before, before)).empty());
}

TEST(Lifecycle, unload_may_leave_a_finalized_plugin_behind)
{
  const Snapshot before = snapshot({});
  Snapshot after = before;
  truth(after, "monitor").state = Lifecycle::Finalized;
  const LifecycleStep unload{LifecycleAction::Unload, "monitor", std::nullopt, std::nullopt};
  EXPECT_TRUE(check_lifecycle(unload, outcome(true, before, after)).empty());
}

TEST(Lifecycle, a_finalized_controller_may_be_loaded_again)
{
  Snapshot before = snapshot({});
  truth(before, "monitor").state = Lifecycle::Finalized;
  const Snapshot after = snapshot({});
  Snapshot loaded = after;
  truth(loaded, "monitor").state = Lifecycle::Unconfigured;
  const LifecycleStep load{LifecycleAction::Load, "monitor", std::nullopt, std::nullopt};
  EXPECT_TRUE(check_lifecycle(load, outcome(true, before, loaded)).empty());
  Snapshot active = before;
  truth(active, "monitor").state = Lifecycle::Active;
  EXPECT_FALSE(check_lifecycle(load, outcome(true, active, loaded)).empty());
}

TEST(Lifecycle, a_load_while_the_previous_callback_runs_must_be_rejected)
{
  const Snapshot before = snapshot({});
  Snapshot after = before;
  truth(after, "odom_pub").state = Lifecycle::Unconfigured;
  Snapshot unloaded = before;
  truth(unloaded, "odom_pub").state = Lifecycle::Unloaded;
  const LifecycleStep load{LifecycleAction::Load, "odom_pub", std::nullopt, std::nullopt};
  EXPECT_FALSE(check_lifecycle(load, outcome(true, unloaded, after), true).empty());
  EXPECT_TRUE(check_lifecycle(load, outcome(true, unloaded, after), false).empty());
  // The abandoned operation may finish meanwhile, a rejection is fine
  EXPECT_TRUE(check_lifecycle(load, outcome(false, before, unloaded), true).empty());
}

TEST(Lifecycle, configure_and_cleanup_follow_the_lifecycle)
{
  Snapshot unconfigured = snapshot({});
  truth(unconfigured, "monitor").state = Lifecycle::Unconfigured;
  const Snapshot inactive = snapshot({});
  const Snapshot active = snapshot({"monitor"});
  const LifecycleStep configure{LifecycleAction::Configure, "monitor", std::nullopt, std::nullopt};
  const LifecycleStep cleanup{LifecycleAction::Cleanup, "monitor", std::nullopt, std::nullopt};
  EXPECT_TRUE(check_lifecycle(configure, outcome(true, unconfigured, inactive)).empty());
  EXPECT_EQ(1u, check_lifecycle(configure, outcome(true, active, inactive)).size());
  EXPECT_EQ(1u, check_lifecycle(configure, outcome(true, unconfigured, unconfigured)).size())
    << "accepted, but not configured";
  EXPECT_TRUE(check_lifecycle(cleanup, outcome(true, inactive, unconfigured)).empty());
  EXPECT_EQ(1u, check_lifecycle(cleanup, outcome(true, active, unconfigured)).size());
}

TEST(Lifecycle, a_rejection_changes_nothing_unless_a_fault_left_an_error_state)
{
  const Snapshot inactive = snapshot({});
  Snapshot unconfigured = snapshot({});
  truth(unconfigured, "monitor").state = Lifecycle::Unconfigured;
  const Snapshot active = snapshot({"monitor"});
  const LifecycleStep cleanup{LifecycleAction::Cleanup, "monitor", std::nullopt, std::nullopt};
  EXPECT_EQ(1u, check_lifecycle(cleanup, outcome(false, inactive, unconfigured)).size());
  EXPECT_TRUE(
    check_lifecycle(cleanup, outcome(false, inactive, unconfigured, {"monitor"})).empty());
  EXPECT_EQ(1u, check_lifecycle(cleanup, outcome(false, inactive, active, {"monitor"})).size())
    << "active is no error state";
}

TEST(Lifecycle, an_unload_rejected_after_a_fault_may_still_finish)
{
  // unload monitor fault=controller:monitor:shutdown:hang, answered as failed on its timeout
  const Snapshot inactive = snapshot({});
  Snapshot unloaded = snapshot({});
  truth(unloaded, "monitor").state = Lifecycle::Unloaded;
  const LifecycleStep unload{LifecycleAction::Unload, "monitor", std::nullopt, std::nullopt};
  EXPECT_TRUE(check_lifecycle(unload, outcome(false, inactive, unloaded, {"monitor"})).empty());
  EXPECT_EQ(1u, check_lifecycle(unload, outcome(false, inactive, unloaded)).size())
    << "without a fault a rejected unload changes nothing";
  const LifecycleStep cleanup{LifecycleAction::Cleanup, "monitor", std::nullopt, std::nullopt};
  EXPECT_EQ(1u, check_lifecycle(cleanup, outcome(false, inactive, unloaded, {"monitor"})).size())
    << "only an unload unloads";
}

TEST(Lifecycle, an_unchecked_lifecycle_step_may_finish_during_the_next_step_but_never_activates)
{
  // unload monitor fault=controller:monitor:shutdown:hang nowait, removed once on_shutdown returns
  const Snapshot inactive = snapshot({});
  Snapshot unloaded = snapshot({});
  truth(unloaded, "monitor").state = Lifecycle::Unloaded;
  Outcome late = outcome(true, inactive, unloaded);
  late.unchecked_lifecycle = {"monitor"};
  EXPECT_TRUE(check_fault(late).empty());
  const LifecycleStep configure{LifecycleAction::Configure, "nav", std::nullopt, std::nullopt};
  EXPECT_TRUE(check_lifecycle(configure, late).empty());
  const LifecycleStep unload{LifecycleAction::Unload, "monitor", std::nullopt, std::nullopt};
  Outcome rejected = outcome(false, inactive, unloaded);
  rejected.unchecked_lifecycle = {"monitor"};
  EXPECT_TRUE(check_lifecycle(unload, rejected).empty());
  EXPECT_FALSE(check_lifecycle(configure, outcome(true, inactive, unloaded)).empty());
  const Snapshot active = snapshot({"monitor"});
  Outcome activated = outcome(true, inactive, active);
  activated.unchecked_lifecycle = {"monitor"};
  EXPECT_FALSE(check_lifecycle(configure, activated).empty());
  EXPECT_FALSE(check_fault(activated).empty());
  // configure nav nowait, then a rejected cleanup nav that leaves nav active
  const LifecycleStep cleanup{LifecycleAction::Cleanup, "nav", std::nullopt, std::nullopt};
  const Snapshot nav_active = snapshot({"nav"});
  Outcome left_active = outcome(false, inactive, nav_active);
  left_active.unchecked_lifecycle = {"nav"};
  EXPECT_FALSE(check_lifecycle(cleanup, left_active).empty());
}

TEST(Lifecycle, a_step_meets_its_expectation_and_changes_nothing_else)
{
  Snapshot before = snapshot({});
  truth(before, "monitor").state = Lifecycle::Unconfigured;
  const Snapshot after = snapshot({});
  LifecycleStep configure{LifecycleAction::Configure, "monitor", std::nullopt, false};
  EXPECT_EQ(1u, check_lifecycle(configure, outcome(true, before, after)).size());
  configure.expect_accepted = std::nullopt;
  Snapshot other_changed = after;
  truth(other_changed, "odom_pub").state = Lifecycle::Unconfigured;
  EXPECT_EQ(1u, check_lifecycle(configure, outcome(true, before, other_changed)).size());
  Snapshot hardware_changed = after;
  truth_hw(hardware_changed, "imu").state = Lifecycle::Inactive;
  EXPECT_EQ(1u, check_lifecycle(configure, outcome(true, before, hardware_changed)).size());
}

TEST(Reported, must_match_the_truth)
{
  Snapshot s = snapshot({"jsb"});
  ReportedState reported;
  for (const auto & c : rig::controllers())
  {
    reported.controllers[c.name] = c.name == "jsb" ? Lifecycle::Active : Lifecycle::Inactive;
  }
  for (const auto & hw : rig::hardware())
  {
    reported.hardware[hw.name] = Lifecycle::Active;
  }
  EXPECT_TRUE(check_reported(s, reported).empty());
  reported.transitioning.insert("jsb");
  EXPECT_EQ(1u, check_reported(s, reported).size());
  reported.transitioning.clear();
  reported.controllers["jsb"] = Lifecycle::Inactive;
  EXPECT_EQ(1u, check_reported(s, reported).size());
  truth(s, "monitor").state = Lifecycle::Finalized;
  reported.controllers.erase("monitor");
  reported.controllers["jsb"] = Lifecycle::Active;
  EXPECT_TRUE(check_reported(s, reported).empty());
}

TEST(Reported, samples_may_only_show_visited_states)
{
  ReportedState sample;
  const uint32_t inactive = 1u << static_cast<uint32_t>(Lifecycle::Inactive);
  const uint32_t active = 1u << static_cast<uint32_t>(Lifecycle::Active);
  for (const auto & c : rig::controllers())
  {
    sample.controllers[c.name] = Lifecycle::Inactive;
  }
  for (const auto & hw : rig::hardware())
  {
    sample.hardware[hw.name] = Lifecycle::Active;
  }
  std::vector<uint32_t> controllers(rig::controllers().size(), inactive);
  const std::vector<uint32_t> hardware(rig::hardware().size(), active);
  EXPECT_TRUE(check_reported_during(sample, controllers, hardware, {}).empty());
  controllers[0] = active;
  EXPECT_EQ(1u, check_reported_during(sample, controllers, hardware, {}).size());
  sample.transitioning.insert(rig::controllers()[0].name);
  EXPECT_TRUE(check_reported_during(sample, controllers, hardware, {}).empty());
}

TEST(Reported, samples_allow_the_alternatives_of_settled_reports)
{
  ReportedState sample;
  const uint32_t inactive = 1u << static_cast<uint32_t>(Lifecycle::Inactive);
  const uint32_t finalized = 1u << static_cast<uint32_t>(Lifecycle::Finalized);
  for (const auto & c : rig::controllers())
  {
    sample.controllers[c.name] = Lifecycle::Inactive;
  }
  for (const auto & hw : rig::hardware())
  {
    sample.hardware[hw.name] = Lifecycle::Active;
  }
  std::vector<uint32_t> controllers(rig::controllers().size(), inactive);
  const std::vector<uint32_t> hardware(
    rig::hardware().size(), 1u << static_cast<uint32_t>(Lifecycle::Active));
  sample.controllers["wrist"] = Lifecycle::Finalized;
  EXPECT_EQ(1u, check_reported_during(sample, controllers, hardware, {}).size());
  EXPECT_TRUE(check_reported_during(sample, controllers, hardware, {"wrist"}).empty());
  controllers[*rig::controller_index("monitor")] = finalized;
  sample.controllers.erase("monitor");
  EXPECT_TRUE(check_reported_during(sample, controllers, hardware, {"wrist"}).empty());
}

TEST(PowerCut, stops_the_unpowered_hardware_and_what_needs_it)
{
  const Names unpowered = {"arm", "base", "imu"};
  // jsb reads every joint with best effort and flipper keeps its power
  EXPECT_TRUE(check_power_cut(snapshot({"jsb", "flipper_pos"}, unpowered)).empty());
  EXPECT_FALSE(check_power_cut(snapshot({"jsb", "flipper_pos"})).empty());
  EXPECT_FALSE(check_power_cut(snapshot({"flipper_pos", "arm_pos"}, unpowered)).empty());
  EXPECT_FALSE(check_power_cut(snapshot({"odom_pub"}, unpowered)).empty());
}

TEST(Shutdown, leaves_nothing_active_or_applied)
{
  const Snapshot down = snapshot({}, {"arm", "base", "flipper", "imu"});
  EXPECT_TRUE(check_shutdown(down).empty());
  Snapshot controller = down;
  truth(controller, "monitor").state = Lifecycle::Active;
  EXPECT_EQ(1u, check_shutdown(controller).size());
  Snapshot hardware = down;
  truth_hw(hardware, "imu").state = Lifecycle::Active;
  EXPECT_EQ(1u, check_shutdown(hardware).size());
  Snapshot applied = down;
  applied.interfaces[*rig::interface_id("f1/position")].live = true;
  EXPECT_EQ(1u, check_shutdown(applied).size());
}

TEST(PowerCut, restoring_brings_back_every_hardware_not_finalized)
{
  EXPECT_TRUE(check_power_restored(snapshot(STARTUP), {}).empty());
  EXPECT_FALSE(check_power_restored(snapshot({}, {"arm"}), {}).empty());
  EXPECT_TRUE(check_power_restored(snapshot({}, {"arm"}), {"arm"}).empty());
  EXPECT_FALSE(check_power_restored(snapshot({}, {"arm", "base"}), {"arm"}).empty());
  Snapshot finalized = snapshot({}, {"arm"});
  finalized.hardware[*rig::hardware_index("arm")].state = Lifecycle::Finalized;
  EXPECT_TRUE(check_power_restored(finalized, {}).empty());
}

}  // namespace cm_test_suite::oracle
