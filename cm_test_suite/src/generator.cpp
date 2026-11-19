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
#include <array>
#include <random>
#include <utility>
#include <vector>

#include "cm_test_suite/oracle.hpp"
#include "cm_test_suite/script.hpp"
#include "names.hpp"

namespace cm_test_suite
{
namespace
{
constexpr std::array ALL_FAULTS = {Fault::Error, Fault::Failure, Fault::Deactivate, Fault::Throw,
                                   Fault::Delay, Fault::Hang,    Fault::Quiet,      Fault::Late};

using oracle::merged;

/// Controllers whose lifecycle callbacks a switch may call.
struct Touched
{
  rig::Names started;
  rig::Names stopped;
  rig::Names chained;
};

Touched touched_by(const SwitchStep & step)
{
  Touched touched;
  touched.started = oracle::from(step.activate);
  if (step.strictness != Strictness::Strict)
  {
    touched.started.merge(rig::closure(touched.started, rig::providers));
  }
  touched.stopped = oracle::from(step.deactivate);
  if (step.strictness == Strictness::ForceAuto)
  {
    touched.stopped.merge(oracle::conflicts_of(touched.started));
  }
  // A provider changes its chained mode when a writer of its references starts or stops
  for (const auto & name : merged(touched.started, touched.stopped))
  {
    touched.chained.merge(rig::written_providers(name));
  }
  return touched;
}

/// Random steps whose faults target hooks the step will actually call.
class Generator
{
public:
  Generator(uint64_t seed, const Capabilities & capabilities)
  : rng_(seed), capabilities_(capabilities)
  {
    strictness_.push_back(Strictness::Strict);
    if (capabilities.auto_switch)
    {
      strictness_.push_back(Strictness::Auto);
    }
    if (capabilities.force_auto_switch)
    {
      strictness_.push_back(Strictness::ForceAuto);
    }
  }

  Step next()
  {
    // The switch belongs to the fault step before it, so it does not count down
    if (std::exchange(switch_next_, false))
    {
      return next_switch();
    }
    // Most of the script runs powered, an unpowered rig rejects nearly everything
    if (power_cut_ && restore_in_-- == 0)
    {
      power_cut_ = false;
      return PowerStep{false, true};
    }
    const double roll = uniform();
    if (roll < 0.56)
    {
      return next_switch();
    }
    if (roll < 0.68)
    {
      return next_fault();
    }
    if (roll < 0.71)
    {
      constexpr std::array PACES = {
        SlowIo::Steady, SlowIo::Jitter, SlowIo::Stall, SlowIo::LongStall};
      return SlowIoStep{PACES[below(PACES.size())]};
    }
    if (roll < 0.85)
    {
      return next_lifecycle();
    }
    if (roll < 0.95 && capabilities_.recover_hardware)
    {
      return next_recover();
    }
    if (can_restore_power() && !power_cut_)
    {
      cut_power();
      return PowerStep{true, !chance(0.3)};
    }
    return next_switch();
  }

  /// A last step that shuts down, sometimes with a fault in the way.
  std::optional<Step> last()
  {
    if (!chance(0.3))
    {
      return std::nullopt;
    }
    ShutdownStep step;
    if (chance(0.5))
    {
      step.fault =
        chance(0.5)
          ? pick_fault(
              TargetKind::Controller, pick_controller(), {Hook::Deactivate, Hook::Shutdown})
          : pick_fault(TargetKind::Hardware, pick_hardware(), {Hook::Deactivate});
    }
    step.stall = chance(0.1);
    return step;
  }

private:
  SwitchStep next_switch()
  {
    SwitchStep step;
    step.strictness = strictness_[below(strictness_.size())];
    const double size = uniform();
    const size_t count = size < 0.7 ? 1 : (size < 0.9 ? 2 + below(2) : 4 + below(4));
    for (size_t i = 0; i < count; ++i)
    {
      const auto name = pick_controller();
      const bool restart = chance(0.05);
      if (restart || chance(0.6))
      {
        step.activate.push_back(name);
      }
      if (restart || step.activate.empty() || step.activate.back() != name)
      {
        step.deactivate.push_back(name);
      }
    }
    if (chance(0.015))
    {
      step.faults = deadline_faults(step);
    }
    else if (chance(0.3))
    {
      const size_t faults = chance(0.8) ? 1 : 2;
      for (size_t i = 0; i < faults; ++i)
      {
        step.faults.push_back({switch_fault(step), static_cast<int>(below(5))});
      }
    }
    step.stall = chance(0.03);
    step.hold = !step.stall && has_sync_hardware() && chance(0.03);
    if (can_restore_power() && !power_cut_ && chance(0.03))
    {
      // A stalled or held loop counts no cycles until the switch returned
      step.power_cut = static_cast<int>(below(step.stall || step.hold ? 2 : 5));
      cut_power();
    }
    return step;
  }

  /// A fault in a hook the switch calls, a lifecycle callback of a controller the switch starts,
  /// stops or chains, an update of a controller chained to a requested one, or a call on the
  /// hardware they command.
  FaultSpec switch_fault(const SwitchStep & step)
  {
    std::vector<std::string> involved = step.activate;
    involved.insert(involved.end(), step.deactivate.begin(), step.deactivate.end());
    const auto name = involved[below(involved.size())];
    if (chance(0.6))
    {
      const auto touched = touched_by(step);
      std::vector<std::pair<Hook, rig::Names>> options = {
        {Hook::Activate, touched.started},
        // A rollback deactivates what the switch started
        {Hook::Deactivate, merged(touched.started, touched.stopped)},
        {Hook::ChainedMode, touched.chained},
        {Hook::Update, rig::connected({name})},
        {Hook::Error, merged(touched.started, touched.stopped)}};
      options.erase(
        std::remove_if(
          options.begin(), options.end(),
          [](const auto & option) { return option.second.empty(); }),
        options.end());
      const auto & [hook, names] = options[below(options.size())];
      const std::vector<std::string> candidates(names.begin(), names.end());
      return pick_fault(TargetKind::Controller, candidates[below(candidates.size())], {hook});
    }
    auto hardware = rig::hardware_of(name);
    if (hardware.empty())
    {
      hardware = {pick_hardware()};
    }
    const std::vector<std::string> candidates(hardware.begin(), hardware.end());
    return pick_fault(
      TargetKind::Hardware, candidates[below(candidates.size())],
      {Hook::Prepare, Hook::Perform, Hook::Read, Hook::Write, Hook::Deactivate, Hook::Error});
  }

  /// A lifecycle callback of a requested controller uses up nearly all of the switch timeout, so
  /// the deadline passes in the slow prepare or perform that follows on the hardware its chain
  /// commands.
  std::vector<TimedFault> deadline_faults(const SwitchStep & step)
  {
    const bool activate = !step.activate.empty() && (step.deactivate.empty() || chance(0.5));
    const auto & names = activate ? step.activate : step.deactivate;
    const auto name = names[below(names.size())];
    rig::Names hardware;
    auto chain = rig::closure({name}, rig::providers);
    chain.insert(name);
    for (const auto & controller : chain)
    {
      hardware.merge(rig::commanded_hardware_of(controller));
    }
    if (hardware.empty())
    {
      hardware = {pick_hardware()};
    }
    const std::vector<std::string> candidates(hardware.begin(), hardware.end());
    const FaultSpec late{
      TargetKind::Controller, name, activate ? Hook::Activate : Hook::Deactivate, Fault::Late};
    const FaultSpec slow{
      TargetKind::Hardware, candidates[below(candidates.size())],
      chance(0.75) ? Hook::Perform : Hook::Prepare, Fault::Delay};
    return {{late, 0}, {slow, 0}};
  }

  /// Sometimes the handling of the fault is slow or hangs, and a switch may arrive meanwhile.
  FaultStep next_fault()
  {
    FaultStep step{cyclic_fault(), {}, true};
    if (chance(0.16))
    {
      const auto & fault = step.fault;
      const Hook hook = fault.kind == TargetKind::Controller || fault.fault == Fault::Deactivate
                          ? Hook::Deactivate
                          : Hook::Error;
      constexpr std::array SLOW = {Fault::Delay, Fault::Late, Fault::Hang};
      step.handling = {FaultSpec{fault.kind, fault.name, hook, SLOW[below(SLOW.size())]}};
      if (chance(0.5))
      {
        step.wait = false;
        switch_next_ = true;
      }
    }
    return step;
  }

  FaultSpec cyclic_fault()
  {
    if (chance(0.5))
    {
      return pick_fault(TargetKind::Hardware, pick_hardware(), {Hook::Read, Hook::Write});
    }
    return pick_fault(TargetKind::Controller, pick_controller(), {Hook::Update});
  }

  LifecycleStep next_lifecycle()
  {
    LifecycleStep step{
      static_cast<LifecycleAction>(below(4)), pick_controller(), std::nullopt, std::nullopt};
    if (chance(0.3) && step.action == LifecycleAction::Configure)
    {
      step.fault =
        pick_fault(TargetKind::Controller, step.controller, {Hook::Configure, Hook::Error});
    }
    if (chance(0.3) && step.action == LifecycleAction::Cleanup)
    {
      step.fault =
        pick_fault(TargetKind::Controller, step.controller, {Hook::Cleanup, Hook::Error});
    }
    if (chance(0.3) && step.action == LifecycleAction::Unload)
    {
      step.fault = pick_fault(TargetKind::Controller, step.controller, {Hook::Shutdown});
    }
    // The next step meets the callback still running
    step.wait = !(step.fault && step.fault->fault == Fault::Hang && chance(0.5));
    return step;
  }

  RecoverStep next_recover()
  {
    RecoverStep step{pick_hardware(), std::nullopt, std::nullopt};
    if (chance(0.3))
    {
      step.fault = pick_fault(
        TargetKind::Hardware, step.hardware,
        {Hook::Configure, Hook::Activate, Hook::Cleanup, Hook::Error});
    }
    return step;
  }

  FaultSpec pick_fault(TargetKind kind, const std::string & name, const std::vector<Hook> & hooks)
  {
    std::vector<std::pair<Hook, Fault>> options;
    for (const auto hook : hooks)
    {
      for (const auto fault : ALL_FAULTS)
      {
        // Hangs and late callbacks are slow, so they are rare
        const bool slow = fault == Fault::Hang || fault == Fault::Late;
        if (applicable(FaultSpec{kind, name, hook, fault}) && (!slow || chance(0.1)))
        {
          options.emplace_back(hook, fault);
        }
      }
    }
    const auto [hook, fault] = options[below(options.size())];
    return {kind, name, hook, fault};
  }

  std::string pick_controller()
  {
    return rig::controllers()[below(rig::controllers().size())].name;
  }

  std::string pick_hardware() { return rig::hardware()[below(rig::hardware().size())].name; }

  /// Plain arithmetic on mt19937_64, whose output the standard fixes, so a seed gives the same
  /// script with every standard library.
  double uniform() { return static_cast<double>(rng_() >> 11) * 0x1.0p-53; }

  bool chance(double probability) { return uniform() < probability; }

  /// The modulo bias is below 2^-50.
  size_t below(size_t count) { return static_cast<size_t>(rng_() % count); }

  /// A cut without the recovery sequence would leave the rest of the script without hardware.
  bool can_restore_power() const { return capabilities_.recover_hardware; }

  /// One to three unpowered steps pass until the restore, not counting a switch that meets a slow
  /// fault handling.
  void cut_power()
  {
    power_cut_ = true;
    restore_in_ = 1 + below(3);
  }

  static bool has_sync_hardware()
  {
    const auto & hardware = rig::hardware();
    return std::any_of(
      hardware.begin(), hardware.end(), [](const rig::Hardware & h) { return !h.is_async; });
  }

  std::mt19937_64 rng_;
  const Capabilities capabilities_;
  std::vector<Strictness> strictness_;
  bool power_cut_ = false;
  size_t restore_in_ = 0;
  bool switch_next_ = false;
};
}  // namespace

Script generate_script(uint64_t seed, size_t steps, const Capabilities & capabilities)
{
  Generator generator(seed, capabilities);
  Script script;
  for (size_t i = 0; i < steps; ++i)
  {
    script.push_back(generator.next());
  }
  if (auto last = generator.last())
  {
    script.push_back(*last);
  }
  return script;
}

}  // namespace cm_test_suite
