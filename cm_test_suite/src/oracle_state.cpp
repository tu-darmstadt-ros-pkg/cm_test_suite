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

#include <optional>

#include "cm_test_suite/oracle.hpp"
#include "names.hpp"

namespace cm_test_suite::oracle
{
namespace
{
bool visited(Lifecycle state, uint32_t mask)
{
  return (mask & (uint32_t{1} << static_cast<uint32_t>(state))) != 0;
}

/// The active controller claiming a hardware command interface, if any.
std::optional<std::string> active_claimer(const Snapshot & snapshot, const std::string & name)
{
  for (size_t i = 0; i < snapshot.controllers.size(); ++i)
  {
    const auto & c = rig::controllers()[i];
    const auto & claims = c.command_interfaces;
    if (
      snapshot.controllers[i].state == Lifecycle::Active &&
      std::find(claims.begin(), claims.end(), name) != claims.end())
    {
      return c.name;
    }
  }
  return std::nullopt;
}

/// Every hardware command interface with the hardware it belongs to.
std::vector<std::pair<std::string, std::string>> hardware_commands()
{
  std::vector<std::pair<std::string, std::string>> commands;
  for (const auto & hw : rig::hardware())
  {
    for (const auto & joint : hw.joints)
    {
      for (const auto & mode : hw.command_modes)
      {
        commands.emplace_back(joint + "/" + mode, hw.name);
      }
    }
  }
  return commands;
}

/// Whether output index of an active controller carries commands of an active writer. A rig
/// chainable controller derives output i from reference i modulo its reference count, so a
/// provider that lost the writer of that reference legitimately holds or writes nothing.
bool commanded(const Snapshot & snapshot, const std::string & controller, size_t index)
{
  const auto & definition = rig::controller(controller);
  if (!definition.chainable)
  {
    return true;
  }
  const std::string reference =
    controller + "/" + definition.references[index % definition.references.size()];
  for (const auto & writer : rig::controllers())
  {
    const auto & claims = writer.command_interfaces;
    const auto it = std::find(claims.begin(), claims.end(), reference);
    if (it != claims.end() && controller_state(snapshot, writer.name) == Lifecycle::Active)
    {
      return commanded(snapshot, writer.name, static_cast<size_t>(it - claims.begin()));
    }
  }
  return !controller_truth(snapshot, controller).chained;
}

/// Whether the active claimer of a hardware command interface is expected to command it.
bool expects_commands(
  const Snapshot & snapshot, const std::string & claimer, const std::string & name)
{
  const auto & claims = rig::controller(claimer).command_interfaces;
  const auto it = std::find(claims.begin(), claims.end(), name);
  return commanded(snapshot, claimer, static_cast<size_t>(it - claims.begin()));
}

/// A controller that refused to stop counts as stopped, and freeing a finalized plugin later is
/// fine, so both have two acceptable reports.
bool may_report(Lifecycle reported, Lifecycle truth, bool refused_stop)
{
  return reported == truth || (refused_stop && reported == Lifecycle::Finalized) ||
         (truth == Lifecycle::Finalized && reported == Lifecycle::Unloaded);
}

bool may_report_visited(Lifecycle reported, uint32_t mask, bool refused_stop)
{
  for (const auto state :
       {Lifecycle::Unloaded, Lifecycle::Unconfigured, Lifecycle::Inactive, Lifecycle::Active,
        Lifecycle::Finalized})
  {
    if (visited(state, mask) && may_report(reported, state, refused_stop))
    {
      return true;
    }
  }
  return false;
}

Lifecycle shown(const std::map<std::string, Lifecycle> & entries, const std::string & name)
{
  const auto it = entries.find(name);
  return it == entries.end() ? Lifecycle::Unloaded : it->second;
}

/// Command interfaces with their claimer, where one claimer ran on active hardware in both
/// snapshots and its chain reaches an active writer.
std::map<std::string, std::string> steady_claims(const Snapshot & begin, const Snapshot & end)
{
  std::map<std::string, std::string> claims;
  for (const auto & [name, hardware] : hardware_commands())
  {
    const auto claimer = active_claimer(end, name);
    const bool steady =
      claimer && active_claimer(begin, name) == claimer &&
      controller_truth(begin, *claimer).epoch == controller_truth(end, *claimer).epoch &&
      hardware_state(begin, hardware) == Lifecycle::Active &&
      hardware_state(end, hardware) == Lifecycle::Active;
    if (steady && expects_commands(end, *claimer, name))
    {
      claims[name] = *claimer;
    }
  }
  return claims;
}
}  // namespace

Snapshot take_snapshot(const Trace & trace)
{
  Snapshot snapshot;
  snapshot.cycle = trace.cycle();
  for (uint16_t i = 0; i < rig::controllers().size(); ++i)
  {
    snapshot.controllers.push_back(trace.controller(i));
  }
  for (uint16_t i = 0; i < rig::hardware().size(); ++i)
  {
    snapshot.hardware.push_back(trace.hardware(i));
  }
  for (uint16_t i = 0; i < rig::interface_count(); ++i)
  {
    snapshot.interfaces.push_back(trace.interface(i));
  }
  return snapshot;
}

bool same_state(const Snapshot & a, const Snapshot & b)
{
  for (size_t i = 0; i < a.controllers.size(); ++i)
  {
    const auto & x = a.controllers[i];
    const auto & y = b.controllers[i];
    if (x.state != y.state || x.epoch != y.epoch || x.chained != y.chained)
    {
      return false;
    }
  }
  for (size_t i = 0; i < a.hardware.size(); ++i)
  {
    if (
      a.hardware[i].state != b.hardware[i].state || a.hardware[i].faulted != b.hardware[i].faulted)
    {
      return false;
    }
  }
  for (size_t i = 0; i < a.interfaces.size(); ++i)
  {
    if (a.interfaces[i].live != b.interfaces[i].live)
    {
      return false;
    }
  }
  return true;
}

Names controllers_in(const Snapshot & snapshot, Lifecycle state)
{
  Names result;
  for (size_t i = 0; i < snapshot.controllers.size(); ++i)
  {
    if (snapshot.controllers[i].state == state)
    {
      result.insert(rig::controllers()[i].name);
    }
  }
  return result;
}

Names loaded(const Snapshot & snapshot)
{
  Names result;
  for (size_t i = 0; i < snapshot.controllers.size(); ++i)
  {
    if (snapshot.controllers[i].state != Lifecycle::Unloaded)
    {
      result.insert(rig::controllers()[i].name);
    }
  }
  return result;
}

Names hardware_in(const Snapshot & snapshot, Lifecycle state)
{
  Names result;
  for (size_t i = 0; i < snapshot.hardware.size(); ++i)
  {
    if (snapshot.hardware[i].state == state)
    {
      result.insert(rig::hardware()[i].name);
    }
  }
  return result;
}

Names affected_by(
  const std::vector<Target> & failed, const Snapshot & before, const Names & started)
{
  const Names active = merged(controllers_in(before, Lifecycle::Active), started);
  Names seeds;
  for (const auto & target : failed)
  {
    if (target.kind == TargetKind::Controller)
    {
      seeds.insert(rig::controllers().at(target.index).name);
      continue;
    }
    for (const auto & name : active)
    {
      if (rig::hardware_of(name).count(rig::hardware().at(target.index).name))
      {
        seeds.insert(name);
      }
    }
  }
  const auto active_neighbours = [&](std::string_view name)
  { return intersection(merged(rig::providers(name), rig::users(name)), active); };
  // A failing callback of an inactive controller gives no reason to stop its active neighbours
  return merged(seeds, rig::closure(intersection(seeds, active), active_neighbours));
}

Findings check_consistency(const Snapshot & snapshot)
{
  Findings findings;
  const Names active = controllers_in(snapshot, Lifecycle::Active);
  for (const auto & name : active)
  {
    for (const auto & other : intersection(rig::claim_conflicts(name), active))
    {
      if (name < other)
      {
        findings.push_back(name + " and " + other + " are active with a shared command interface");
      }
    }
    const Names missing = without(rig::providers(name), active);
    if (!missing.empty())
    {
      findings.push_back(name + " is active while its providers " + text(missing) + " are not");
    }
    for (const auto & hardware : rig::hardware_of(name))
    {
      if (!hardware_usable(name, hardware, snapshot))
      {
        findings.push_back(
          name + " is active while its hardware " + hardware + " is " +
          to_string(hardware_state(snapshot, hardware)));
      }
    }
    const auto & truth = controller_truth(snapshot, name);
    if (truth.failed_epoch == truth.epoch)
    {
      findings.push_back(name + " is still active after its update failed");
    }
    if (rig::controller(name).chainable)
    {
      const bool written = !intersection(rig::reference_writers(name), active).empty();
      if (truth.chained != written)
      {
        findings.push_back(
          name + (truth.chained ? " is in chained mode without an active writer"
                                : " is not in chained mode although a writer is active"));
      }
    }
  }
  for (const auto & hw : rig::hardware())
  {
    if (hardware_truth(snapshot, hw.name).faulted)
    {
      findings.push_back("hardware " + hw.name + " failed but was not taken down");
    }
  }
  for (const auto & command : hardware_commands())
  {
    const auto & name = command.first;
    const bool live = snapshot.interfaces.at(*rig::interface_id(name)).live;
    const auto claimer = active_claimer(snapshot, name);
    if (live && !claimer)
    {
      findings.push_back(name + " is applied by the hardware without an active claimer");
    }
  }
  return findings;
}

Names state_readers(const Snapshot & snapshot, const std::string & hardware)
{
  Names readers;
  for (const auto & name : controllers_in(snapshot, Lifecycle::Active))
  {
    if (rig::hardware_of(name).count(hardware) && !rig::commanded_hardware_of(name).count(hardware))
    {
      readers.insert(name);
    }
  }
  return readers;
}

Names hardware_without_io(const Snapshot & begin, const Snapshot & end)
{
  Names names;
  for (const auto & hw : rig::hardware())
  {
    const auto & first = hardware_truth(begin, hw.name);
    const auto & last = hardware_truth(end, hw.name);
    const bool active = first.state == Lifecycle::Active && last.state == Lifecycle::Active;
    // Inactive hardware is read but not written, like upstream, as long as a controller reads it
    const bool inactive_and_read = first.state == Lifecycle::Inactive &&
                                   last.state == Lifecycle::Inactive &&
                                   !state_readers(end, hw.name).empty();
    const bool unread = last.reads == first.reads;
    if ((active && (unread || last.writes == first.writes)) || (inactive_and_read && unread))
    {
      names.insert(hw.name);
    }
  }
  return names;
}

Findings check_liveness(const Snapshot & begin, const Snapshot & end)
{
  Findings findings;
  for (const auto & name : hardware_without_io(begin, end))
  {
    if (hardware_state(end, name) == Lifecycle::Active)
    {
      findings.push_back("active hardware " + name + " was not read and written while settling");
    }
    else
    {
      findings.push_back(
        "inactive hardware " + name + " was not read while settling although " +
        text(state_readers(end, name)) + " read its state");
    }
  }
  for (const auto & [name, claimer] : steady_claims(begin, end))
  {
    const auto & interface = end.interfaces.at(*rig::interface_id(name));
    if (!interface.live)
    {
      findings.push_back(
        name + " is claimed by " + claimer + " but the hardware does not apply it");
    }
    else if (interface.applied_cycle <= begin.cycle)
    {
      findings.push_back(
        "no command of " + claimer + " reached " + name + " while settling, " +
        std::to_string(end.cycle - begin.cycle) + " cycles");
    }
  }
  for (const auto & name : controllers_in(end, Lifecycle::Active))
  {
    const auto & first = controller_truth(begin, name);
    const auto & last = controller_truth(end, name);
    if (
      first.state == Lifecycle::Active && first.epoch == last.epoch &&
      first.updates == last.updates)
    {
      findings.push_back(
        "active controller " + name + " was not updated while settling, " +
        std::to_string(end.cycle - begin.cycle) + " cycles");
    }
  }
  return findings;
}

Names unapplied_claims(const Snapshot & begin, const Snapshot & end)
{
  Names names;
  for (const auto & [name, claimer] : steady_claims(begin, end))
  {
    if (!end.interfaces.at(*rig::interface_id(name)).live)
    {
      names.insert(name);
    }
  }
  return names;
}

Findings check_reported(const Snapshot & snapshot, const ReportedState & reported)
{
  Findings findings;
  for (const auto & name : reported.transitioning)
  {
    findings.push_back(name + " is reported in a transition after the step settled");
  }
  for (const auto & c : rig::controllers())
  {
    const auto & truth = controller_truth(snapshot, c.name);
    const auto state = shown(reported.controllers, c.name);
    if (!may_report(state, truth.state, truth.refused_stop))
    {
      findings.push_back(
        "controller " + c.name + " is reported " + to_string(state) + " but is " +
        to_string(truth.state));
    }
  }
  for (const auto & hw : rig::hardware())
  {
    const auto truth = hardware_state(snapshot, hw.name);
    const auto state = shown(reported.hardware, hw.name);
    if (state != truth)
    {
      findings.push_back(
        "hardware " + hw.name + " is reported " + to_string(state) + " but is " + to_string(truth));
    }
  }
  for (const auto & [name, state] : reported.controllers)
  {
    if (!rig::controller_index(name))
    {
      findings.push_back("unknown controller " + name + " is reported");
    }
  }
  return findings;
}

Findings check_reported_during(
  const ReportedState & sample, const std::vector<uint32_t> & visited_controllers,
  const std::vector<uint32_t> & visited_hardware, const Names & refused_stop)
{
  Findings findings;
  for (size_t i = 0; i < rig::controllers().size(); ++i)
  {
    const auto & name = rig::controllers()[i].name;
    const auto state = shown(sample.controllers, name);
    const bool refused = refused_stop.count(name) > 0;
    if (
      !sample.transitioning.count(name) &&
      !may_report_visited(state, visited_controllers.at(i), refused))
    {
      findings.push_back(
        "controller " + name + " was reported " + to_string(state) +
        " during the step but never was in that state");
    }
  }
  for (size_t i = 0; i < rig::hardware().size(); ++i)
  {
    const auto & name = rig::hardware()[i].name;
    const auto state = shown(sample.hardware, name);
    if (!sample.transitioning.count(name) && !visited(state, visited_hardware.at(i)))
    {
      findings.push_back(
        "hardware " + name + " was reported " + to_string(state) +
        " during the step but never was in that state");
    }
  }
  return findings;
}

Findings check_startup(const Snapshot & snapshot, const rig::StartupProfile & profile)
{
  Findings findings;
  if (loaded(snapshot) != profile.controllers_loaded)
  {
    findings.push_back(
      "loaded " + text(loaded(snapshot)) + " instead of " + text(profile.controllers_loaded));
  }
  const Names active = controllers_in(snapshot, Lifecycle::Active);
  if (active != profile.controllers_active)
  {
    findings.push_back(
      "active " + text(active) + " instead of " + text(profile.controllers_active));
  }
  const Names inactive_hardware =
    without(profile.hardware_active, hardware_in(snapshot, Lifecycle::Active));
  if (!inactive_hardware.empty())
  {
    findings.push_back("startup hardware " + text(inactive_hardware) + " is not active");
  }
  return findings;
}

Names running_without_power(const Snapshot & snapshot)
{
  Names unpowered;
  for (const auto & hw : rig::hardware())
  {
    if (hw.faults_on_power_loss)
    {
      unpowered.insert(hw.name);
    }
  }
  Names running;
  for (const auto & name : controllers_in(snapshot, Lifecycle::Active))
  {
    const Names used = rig::controller(name).best_effort_states ? rig::commanded_hardware_of(name)
                                                                : rig::hardware_of(name);
    if (!intersection(used, unpowered).empty())
    {
      running.insert(name);
    }
  }
  return running;
}

Findings check_power_cut(const Snapshot & after)
{
  Findings findings;
  for (const auto & hw : rig::hardware())
  {
    if (hw.faults_on_power_loss && hardware_state(after, hw.name) == Lifecycle::Active)
    {
      findings.push_back(hw.name + " is still active without power");
    }
  }
  const Names running = running_without_power(after);
  if (!running.empty())
  {
    findings.push_back(text(running) + " are still active on hardware without power");
  }
  return findings;
}

Findings check_power_restored(const Snapshot & after, const Names & not_attempted)
{
  Names down;
  for (const auto & hw : rig::hardware())
  {
    const auto state = hardware_state(after, hw.name);
    if (
      state != Lifecycle::Active && state != Lifecycle::Finalized && !not_attempted.count(hw.name))
    {
      down.insert(hw.name);
    }
  }
  if (down.empty())
  {
    return {};
  }
  return {text(down) + " did not recover after the power came back"};
}

Findings check_shutdown(const Snapshot & after)
{
  Findings findings;
  const Names active = controllers_in(after, Lifecycle::Active);
  if (!active.empty())
  {
    findings.push_back(text(active) + " are still active after shutdown");
  }
  const Names hardware = hardware_in(after, Lifecycle::Active);
  if (!hardware.empty())
  {
    findings.push_back("hardware " + text(hardware) + " is still active after shutdown");
  }
  for (uint16_t i = 0; i < after.interfaces.size(); ++i)
  {
    if (after.interfaces[i].live)
    {
      findings.push_back(rig::interface_name(i) + " is still applied after shutdown");
    }
  }
  return findings;
}

}  // namespace cm_test_suite::oracle
