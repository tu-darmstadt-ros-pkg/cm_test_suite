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

#include "cm_test_suite/runner.hpp"

#include <algorithm>
#include <atomic>
#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <thread>
#include <utility>

#include "cm_test_suite/oracle.hpp"
#include "cm_test_suite/rt_allocations.hpp"
#include "cm_test_suite/trace.hpp"
#include "control_loop.hpp"
#include "event_summary.hpp"
#include "operation.hpp"
#include "overloaded.hpp"
#include "status_sampler.hpp"

namespace cm_test_suite
{
namespace
{
using Clock = ControlLoop::Clock;
using oracle::Names;
using oracle::Snapshot;
using std::chrono::milliseconds;

constexpr std::chrono::seconds HANG_MARGIN{3};
/// How long the list services and the activity topic may lag behind the settled truth.
constexpr milliseconds REPORT_DEADLINE{500};
/// Cycles per settle round between two snapshots.
constexpr size_t SETTLE_CYCLES = 10;
/// How long the truth must stay unchanged to count as settled. Stepped cycles take microseconds,
/// and a controller manager finishes a fault off RT, so cycles alone do not give it time. 10
/// threaded cycles take about as long.
constexpr milliseconds QUIET_WINDOW{10};
/// How long a controller manager may take to finish a component after its hang returned.
constexpr std::chrono::seconds SHUTDOWN_DEADLINE{1};
/// How long settling waits for hardware that runs on its own clock to be read and written.
constexpr milliseconds IO_DEADLINE{100};
/// How long a fault armed for a cyclic call may take to fire on active hardware or controllers.
constexpr milliseconds FIRING_DEADLINE{100};
/// How long a CM may take to bring up the startup profile by itself. The setup check judges only
/// the outcome, and loading every startup controller takes seconds on a starved CI machine.
constexpr std::chrono::seconds STARTUP_DEADLINE{10};
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
/// No fault delays a cycle on purpose, so a long cycle means the controller manager blocked it.
constexpr milliseconds LONGEST_CYCLE{200};
/// How long an inconsistency or an unapplied claim may last while settling, one supervisor step.
constexpr milliseconds CONSISTENCY_LAG{1000};
#else
constexpr milliseconds LONGEST_CYCLE{20};
constexpr milliseconds CONSISTENCY_LAG{200};
#endif

Target target_of(const FaultSpec & fault)
{
  const auto index = fault.kind == TargetKind::Controller ? rig::controller_index(fault.name)
                                                          : rig::hardware_index(fault.name);
  return {fault.kind, *index};
}

std::string fault_kind(const FaultSpec & fault)
{
  return std::string(to_string(fault.hook)) + ":" + to_string(fault.fault);
}

std::string milliseconds_text(Clock::duration duration)
{
  return std::to_string(std::chrono::duration_cast<milliseconds>(duration).count()) + " ms";
}

/// The hardware a power cut faults.
std::vector<Target> unpowered_hardware()
{
  std::vector<Target> targets;
  for (uint16_t i = 0; i < rig::hardware().size(); ++i)
  {
    if (rig::hardware()[i].faults_on_power_loss)
    {
      targets.push_back({TargetKind::Hardware, i});
    }
  }
  return targets;
}

/// The two last snapshots of settling, between which nothing changed.
struct Window
{
  Snapshot begin;
  Snapshot end;
};

class Session
{
public:
  Session(const CmFactory & factory, const RunOptions & options)
  : factory_(factory), options_(options)
  {
  }

  RunResult run(const Script & script)
  {
    trace().reset();
    faults().clear();
    if (start())
    {
      for (size_t i = 0; i < script.size() && result_.passed(); ++i)
      {
        context_ = "step " + std::to_string(i + 1) + " '" + to_text(script[i]) + "'";
        execute(script[i]);
      }
    }
    finish();
    return std::move(result_);
  }

  RunResult run_rejected(const std::string & description)
  {
    trace().reset();
    faults().clear();
    context_ = "setup";
    CmConfig config = options_.config;
    config.robot_description = description;
    try
    {
      cm_ = factory_(config);
    }
    catch (const std::exception &)
    {
      // Rejected loudly
      return std::move(result_);
    }
    loop_ = std::make_unique<ControlLoop>(*cm_, options_.mode);
    // Long enough for a CM on an idle machine to activate a description it wrongly accepted
    const auto deadline = Clock::now() + 2 * options_.config.switch_timeout;
    while (Clock::now() < deadline)
    {
      loop_->tick();
    }
    for (const auto & error : loop_->take_errors())
    {
      fail(check::EXCEPTION, "cycle() threw: " + error);
    }
    const Snapshot now = oracle::take_snapshot(trace());
    for (const auto state : {Lifecycle::Inactive, Lifecycle::Active})
    {
      for (const auto & name : oracle::hardware_in(now, state))
      {
        fail(
          check::OUTCOME,
          "accepted a description with duplicate interfaces, " + name + " is " + to_string(state));
      }
    }
    try
    {
      cm_->reported();
    }
    catch (const std::exception & e)
    {
      fail(check::EXCEPTION, std::string("reporting the state threw: ") + e.what());
    }
    finish();
    return std::move(result_);
  }

private:
  bool start()
  {
    context_ = "setup";
    try
    {
      cm_ = factory_(options_.config);
    }
    catch (const std::exception & e)
    {
      fail(check::SETUP, std::string("creating the controller manager threw: ") + e.what());
      return false;
    }
    loop_ = std::make_unique<ControlLoop>(*cm_, options_.mode);
    const auto & profile = options_.config.startup;
    if (options_.capabilities.startup)
    {
      await_startup(profile);
    }
    else
    {
      for (const auto & name : profile.controllers_loaded)
      {
        expect_accepted([&] { return cm_->load(name); }, "load " + name);
        expect_accepted([&] { return cm_->configure(name); }, "configure " + name);
      }
      const std::vector<std::string> active(
        profile.controllers_active.begin(), profile.controllers_active.end());
      expect_accepted(
        [&] { return cm_->switch_controllers(active, {}, Strictness::Strict); },
        "activate the startup controllers");
    }
    const Window window = settle();
    report(check::SETUP, oracle::check_startup(window.end, profile));
    check_settled(window);
    loop_->take_longest_cycle();
    return result_.passed();
  }

  /// The startup switch is done once the truth reached the profile and the controller manager
  /// reports it, because the truth is reached before the switch's last steps (rulebook section 4).
  bool startup_reported(const rig::StartupProfile & profile)
  {
    const Snapshot now = oracle::take_snapshot(trace());
    if (!oracle::check_startup(now, profile).empty())
    {
      return false;
    }
    try
    {
      return oracle::check_reported(now, cm_->reported()).empty();
    }
    catch (const std::exception &)
    {
      // check_settled reports the exception
      return true;
    }
  }

  /// The controller manager brings up the startup profile by itself, settling checks it afterwards.
  void await_startup(const rig::StartupProfile & profile)
  {
    const auto deadline = Clock::now() + STARTUP_DEADLINE;
    while (!startup_reported(profile) && Clock::now() < deadline)
    {
      loop_->tick();
    }
  }

  void execute(const Step & step)
  {
    if (const auto * power = std::get_if<PowerStep>(&step); power && !power->cut)
    {
      restore_power();
      return;
    }
    const Snapshot before = oracle::take_snapshot(trace());
    // The states an unchecked step passed through belong to this step
    const std::optional<Snapshot> unchecked = std::exchange(unchecked_before_, std::nullopt);
    Names unchecked_lifecycle = std::exchange(unchecked_lifecycle_, {});
    if (!unchecked)
    {
      reset_visited();
    }
    const uint64_t allocations = rt_allocations();
    loop_->take_longest_cycle();
    Names hardware_excused;
    const bool power_was_cut = power_cut_;
    StatusSampler sampler(*cm_);
    const auto * lifecycle = std::get_if<LifecycleStep>(&step);
    // A callback of the previous instance, for example an abandoned on_cleanup, still runs
    const bool previous_callback_running =
      lifecycle && trace().controller_hanging(*rig::controller_index(lifecycle->controller));
    const OperationResult operation = std::visit(
      Overloaded{
        [&](const SwitchStep & s)
        {
          operation_returned_ = false;
          return operate(
            [&]
            {
              const bool accepted = cm_->switch_controllers(s.activate, s.deactivate, s.strictness);
              operation_returned_ = true;
              trace().release_reads();
              return accepted;
            },
            switch_actions(s));
        },
        [&](const FaultStep & s) { return inject(s); },
        [&](const LifecycleStep & s) { return operate(lifecycle_call(s), fault_action(s.fault)); },
        [&](const RecoverStep & s)
        {
          hardware_excused.insert(s.hardware);
          return operate([&] { return cm_->recover_hardware(s.hardware); }, fault_action(s.fault));
        },
        [&](const PowerStep & s) { return cut_power(s); }, [&](const SlowIoStep & s)
        { return slow_io(s.pace); }, [&](const ShutdownStep & s) { return shut_down(s); }},
      step);
    trace().release_reads();
    if (!operation.accepted)
    {
      stop_sampling(sampler);
      fail(operation.hung ? check::HANG : check::EXCEPTION, operation.error);
      return;
    }
    log(*operation.accepted ? "accepted" : "rejected");
    count_answer(step, *operation.accepted);
    if (power_was_cut || power_cut_)
    {
      ++result_.stats.unpowered_steps;
    }
    const auto * fault = std::get_if<FaultStep>(&step);
    if ((lifecycle && !lifecycle->wait) || (fault && !fault->wait))
    {
      stop_sampling(sampler);
      unchecked_before_ = unchecked.value_or(before);
      unchecked_lifecycle_ = std::move(unchecked_lifecycle);
      if (lifecycle)
      {
        unchecked_lifecycle_.insert(lifecycle->controller);
      }
      return;
    }
    if (std::holds_alternative<ShutdownStep>(step))
    {
      stop_sampling(sampler);
      report(check::SHUTDOWN, shutdown_findings());
      report_events();
      return;
    }
    check_power_reaction();
    const Window window = settle();
    stop_sampling(sampler);
    check_samples(sampler, refused_stop(before, window.end));
    const auto fired = faults().take_fired();
    clear_faults();
    check_settled(window);
    // A held cycle is long on purpose
    const auto * held = std::get_if<SwitchStep>(&step);
    check_real_time(allocations, !(held && held->hold));
    const Snapshot & after = window.end;

    const auto * switch_step = std::get_if<SwitchStep>(&step);
    const auto timeout = options_.config.switch_timeout + TIMEOUT_MARGIN;
    if (switch_step && operation.duration > timeout)
    {
      fail(check::TIMEOUT, "the switch took " + milliseconds_text(operation.duration));
    }
    Names started;
    if (switch_step)
    {
      const Names activate(switch_step->activate.begin(), switch_step->activate.end());
      started = rig::closure(activate, rig::providers);
      started.insert(activate.begin(), activate.end());
    }
    const bool unpowered = power_was_cut || power_cut_;
    std::vector<Target> failed = fired;
    for (const auto hardware : trace().take_held())
    {
      failed.push_back({TargetKind::Hardware, hardware});
    }
    if (unpowered)
    {
      const auto cut = unpowered_hardware();
      failed.insert(failed.end(), cut.begin(), cut.end());
    }
    Names excused = oracle::affected_by(failed, before, started);
    if (unchecked)
    {
      excused.merge(oracle::affected_by(failed, *unchecked, started));
    }
    for (const auto & target : failed)
    {
      if (target.kind == TargetKind::Hardware)
      {
        hardware_excused.insert(rig::hardware().at(target.index).name);
      }
    }
    const oracle::Outcome outcome{
      *operation.accepted,
      before,
      after,
      excused,
      hardware_excused,
      operation.duration >= options_.config.switch_timeout,
      unchecked_lifecycle};
    std::visit(
      Overloaded{
        [&](const SwitchStep & s) { report(check::OUTCOME, oracle::check_switch(s, outcome)); },
        [&](const FaultStep &) { report(check::OUTCOME, oracle::check_fault(outcome)); },
        [&](const LifecycleStep & s)
        { report(check::OUTCOME, oracle::check_lifecycle(s, outcome, previous_callback_running)); },
        [&](const RecoverStep & s) { report(check::OUTCOME, oracle::check_recover(s, outcome)); },
        [&](const PowerStep & s)
        {
          if (s.wait)
          {
            report(check::POWER_CUT, oracle::check_power_cut(after));
          }
          report(check::OUTCOME, oracle::check_fault(outcome));
        },
        [&](const SlowIoStep &) { report(check::OUTCOME, oracle::check_fault(outcome)); },
        [&](const ShutdownStep &) {}},
      step);
  }

  std::vector<TimedAction> switch_actions(const SwitchStep & step)
  {
    std::vector<TimedAction> actions;
    for (const auto & timed : step.faults)
    {
      actions.push_back(
        {static_cast<uint64_t>(timed.after_cycles), [this, fault = timed.fault] { arm(fault); }});
    }
    if (step.stall)
    {
      actions.push_back(stall_action());
    }
    if (step.hold)
    {
      actions.push_back(hold_action());
    }
    if (step.power_cut)
    {
      // A later cut runs after the stall or hold ended, a cut at 0 meets a loop without cycles
      const bool cycling = *step.power_cut > 0 || (!step.stall && !step.hold);
      actions.push_back(
        {static_cast<uint64_t>(*step.power_cut), [this, cycling] { set_power(false, cycling); }});
    }
    return actions;
  }

  /// Without wait, cycles until the faults were taken, so the next step meets their handling. A
  /// fault on something inactive is never taken, so its wait is short.
  OperationResult inject(const FaultStep & step)
  {
    arm(step.fault);
    for (const auto & handling : step.handling)
    {
      arm(handling);
    }
    const auto await_taken = [&](const FaultSpec & fault, Clock::duration limit)
    {
      const auto deadline = Clock::now() + limit;
      while (faults().pending(target_of(fault), fault.hook) && Clock::now() < deadline)
      {
        loop_->tick();
      }
      return !faults().pending(target_of(fault), fault.hook);
    };
    if (!step.wait && await_taken(step.fault, FIRING_DEADLINE) && !step.handling.empty())
    {
      await_taken(step.handling.front(), options_.config.switch_timeout);
    }
    OperationResult result;
    result.accepted = true;
    return result;
  }

  std::vector<TimedAction> fault_action(const std::optional<FaultSpec> & fault)
  {
    if (!fault)
    {
      return {};
    }
    return {{0, [this, spec = *fault] { arm(spec); }}};
  }

  std::function<bool()> lifecycle_call(const LifecycleStep & step)
  {
    switch (step.action)
    {
      case LifecycleAction::Load:
        return [this, name = step.controller] { return cm_->load(name); };
      case LifecycleAction::Configure:
        return [this, name = step.controller] { return cm_->configure(name); };
      case LifecycleAction::Cleanup:
        return [this, name = step.controller] { return cm_->cleanup(name); };
      case LifecycleAction::Unload:
        return [this, name = step.controller] { return cm_->unload(name); };
    }
    throw std::logic_error("Unknown lifecycle action");
  }

  void set_power(bool on, bool watch_reaction = false)
  {
    power_cut_ = !on;
    trace().set_power(on);
    if (watch_reaction)
    {
      const auto cut = Clock::now();
      power_reaction_ = std::async(
        std::launch::async,
        [cut]() -> std::optional<std::string>
        {
          // A hung or late callback cannot be cut short, so its return restarts the bound
          std::vector<Clock::time_point> since(rig::controllers().size(), cut);
          // A controller past its bound may report its stop late, but must never be updated again
          std::vector<std::optional<uint64_t>> held(since.size());
          const auto deadline = cut + POWER_CUT_REACTION + HANG + std::chrono::seconds(1);
          uint64_t cycle = trace().cycle();
          auto cycled = cut;
          while (true)
          {
            // Also a controller not running yet, which a late on_activate may still start
            const auto now = Clock::now();
            for (uint16_t index = 0; index < since.size(); ++index)
            {
              if (trace().controller_hanging(index))
              {
                since[index] = now;
                held[index].reset();
              }
            }
            const Snapshot snapshot = oracle::take_snapshot(trace());
            if (snapshot.cycle != cycle)
            {
              cycle = snapshot.cycle;
              cycled = now;
            }
            const Names running = oracle::running_without_power(snapshot);
            for (const auto & name : running)
            {
              const uint16_t index = *rig::controller_index(name);
              const uint64_t updates = snapshot.controllers[index].updates;
              if (now - since[index] <= POWER_CUT_REACTION)
              {
                continue;
              }
              if (!held[index])
              {
                held[index] = updates;
              }
              else if (updates != *held[index])
              {
                return "a controller without power was still updated " +
                       milliseconds_text(now - since[index]) +
                       " after the cut or the return of their hung callback";
              }
            }
            if (running.empty())
            {
              return std::nullopt;
            }
            if (now > deadline)
            {
              if (now - cycled > POWER_CUT_REACTION)
              {
                return std::string(
                  "the control loop stopped cycling while controllers without power were active");
              }
              return std::nullopt;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
        });
    }
  }

  void check_power_reaction()
  {
    if (!power_reaction_.valid())
    {
      return;
    }
    // Stepped cycles run only on this thread
    while (power_reaction_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
    {
      loop_->tick();
    }
    if (const auto finding = power_reaction_.get())
    {
      fail(check::TIMEOUT, *finding);
    }
  }

  /// The control loop keeps cycling while async hardware is slow, so a controller manager that
  /// counts a slow async cycle as hung faults it, which check_fault reports.
  OperationResult slow_io(SlowIo pace)
  {
    return operate(
      [pace]
      {
        trace().set_slow_io(pace);
        std::this_thread::sleep_for(SLOW_IO_TIME);
        trace().set_slow_io(SlowIo::Off);
        return true;
      },
      {});
  }

  /// Waits until the controller manager took every unpowered component out of the active state.
  OperationResult cut_power(const PowerStep & step)
  {
    auto result = operate(
      [&]
      {
        set_power(false, true);
        return true;
      },
      {});
    if (!result.accepted || !step.wait)
    {
      return result;
    }
    const auto deadline = Clock::now() + 3 * options_.config.switch_timeout;
    while (!oracle::check_power_cut(oracle::take_snapshot(trace())).empty() &&
           Clock::now() < deadline)
    {
      loop_->tick();
    }
    return result;
  }

  /// The recovery sequence of hector_controller_spawner, each call checked as its own step.
  void restore_power()
  {
    set_power(true);
    settle();
    const std::string context = context_;
    ReportedState reported;
    try
    {
      reported = cm_->reported();
    }
    catch (const std::exception & e)
    {
      fail(check::EXCEPTION, std::string("reporting the state threw: ") + e.what());
      return;
    }
    Names not_attempted;
    for (const auto & [name, state] : reported.hardware)
    {
      if (state == Lifecycle::Active || state == Lifecycle::Finalized)
      {
        continue;
      }
      if (!result_.passed())
      {
        not_attempted.insert(name);
        continue;
      }
      const RecoverStep recover{name, std::nullopt, true};
      context_ = context + " then '" + to_text(recover) + "'";
      execute(recover);
    }
    SwitchStep restart;
    restart.strictness = options_.capabilities.auto_switch ? Strictness::Auto : Strictness::Strict;
    for (const auto & name : options_.config.startup.controllers_active)
    {
      const auto it = reported.controllers.find(name);
      if (it == reported.controllers.end() || it->second != Lifecycle::Active)
      {
        restart.activate.push_back(name);
      }
    }
    if (!restart.activate.empty() && result_.passed())
    {
      context_ = context + " then '" + to_text(restart) + "'";
      execute(restart);
    }
    context_ = context;
    report(
      check::POWER_RECOVERY,
      oracle::check_power_restored(oracle::take_snapshot(trace()), not_attempted));
  }

  /// Stops the control loop until the operation returned, at most for its time limit. An operation
  /// that waits for the stalled loop still returns only after the limit.
  TimedAction stall_action()
  {
    return {
      1, [this]
      {
        loop_->stall(
          options_.config.switch_timeout + TIMEOUT_MARGIN,
          [this] { return operation_returned_.load(); });
      }};
  }

  /// Holds the control loop inside a read of sync hardware until the operation returned, at most
  /// for its time limit. Like a stall it blocks the harness, so later actions run after the hold.
  /// A fault or cut behind it thus never lands during the held read. In stepped mode the harness
  /// thread itself runs the held cycle.
  TimedAction hold_action()
  {
    return {
      1, [this]
      {
        const auto until = Clock::now() + options_.config.switch_timeout + TIMEOUT_MARGIN;
        trace().hold_reads(until);
        while (!operation_returned_ && Clock::now() < until)
        {
          loop_->tick();
        }
        // A late hold may have armed after the switch released the reads
        trace().release_reads();
        // The loop may still sit in the held read, so the actions behind wait for its end
        const uint64_t held = trace().cycle();
        while (trace().cycle() == held && Clock::now() < until)
        {
          loop_->tick();
        }
      }};
  }

  OperationResult shut_down(const ShutdownStep & step)
  {
    auto actions = fault_action(step.fault);
    if (step.stall)
    {
      actions.push_back(stall_action());
    }
    operation_returned_ = false;
    auto result = operate(
      [&]
      {
        cm_->shutdown();
        operation_returned_ = true;
        // cycle() is never called after shutdown() returned (cm_under_test.hpp)
        loop_->stop();
        return true;
      },
      std::move(actions));
    loop_->stop();
    // Sampled after shutdown returned, so a hang ending about a cycle before may still check early
    hang_outlived_shutdown_ = trace().hangs_running() > 0;
    wait_for_hangs();
    shut_down_ = true;
    if (result.duration > options_.config.switch_timeout + TIMEOUT_MARGIN)
    {
      fail(check::TIMEOUT, "shutdown took " + milliseconds_text(result.duration));
    }
    return result;
  }

  OperationResult operate(const std::function<bool()> & call, std::vector<TimedAction> actions)
  {
    auto result = run_operation(
      *loop_, call, std::move(actions), options_.config.switch_timeout + HANG + HANG_MARGIN);
    for (const auto & error : loop_->take_errors())
    {
      fail(check::EXCEPTION, "cycle() threw: " + error);
    }
    return result;
  }

  /// Keeps cycling until no injected hang sleeps in a callback any more.
  void wait_for_hangs()
  {
    const auto deadline = Clock::now() + HANG + std::chrono::seconds(1);
    while (trace().hangs_running() > 0 && Clock::now() < deadline)
    {
      loop_->tick();
    }
  }

  /// Polls the shutdown check without cycles, as no cycle runs after shutdown. A controller manager
  /// may finish a component whose hang returned only after the injected hang counted as over.
  oracle::Findings shutdown_findings()
  {
    const auto deadline =
      Clock::now() + (hang_outlived_shutdown_ ? SHUTDOWN_DEADLINE : Clock::duration::zero());
    while (true)
    {
      auto findings = oracle::check_shutdown(oracle::take_snapshot(trace()));
      if (findings.empty() || Clock::now() > deadline)
      {
        return findings;
      }
      std::this_thread::sleep_for(milliseconds(1));
    }
  }

  /// Cycles until no hang runs and the truth stopped changing, bounded by the longest hang.
  Window settle()
  {
    wait_for_hangs();
    auto deadline = Clock::now() + HANG + std::chrono::seconds(1);
    Snapshot previous = oracle::take_snapshot(trace());
    auto quiet_since = Clock::now();
    std::optional<Clock::time_point> lagging_since;
    while (true)
    {
      // A fault that fires while settling can start a new hang, which is waited out as well
      if (trace().hangs_running() > 0)
      {
        wait_for_hangs();
        deadline = std::max(deadline, Clock::now() + std::chrono::seconds(1));
        previous = oracle::take_snapshot(trace());
        quiet_since = Clock::now();
      }
      loop_->run_cycles(SETTLE_CYCLES);
      await_io(previous);
      Snapshot current = oracle::take_snapshot(trace());
      const bool faulted = std::any_of(
        current.hardware.begin(), current.hardware.end(),
        [](const HardwareTruth & hw) { return hw.faulted; });
      const bool failed_running = std::any_of(
        current.controllers.begin(), current.controllers.end(), [](const ControllerTruth & c)
        { return c.state == Lifecycle::Active && c.failed_epoch == c.epoch; });
      // A CM may still be between two teardown steps while the chain is inconsistent, but no
      // longer, a lasting inconsistency is reported
      const bool consistent =
        !recorded(check::CONSISTENCY) || oracle::check_consistency(current).empty();
      // Likewise a CM may start the interfaces of a claimer a supervisor step after activating it
      const bool applied =
        !recorded(check::LIVENESS) || oracle::unapplied_claims(previous, current).empty();
      if (consistent && applied)
      {
        lagging_since.reset();
      }
      else if (!lagging_since)
      {
        lagging_since = Clock::now();
      }
      const bool catching_up = lagging_since && Clock::now() - *lagging_since < CONSISTENCY_LAG;
      // A lifecycle or mode switch callback still running means the step is not done yet
      const std::vector<std::pair<bool, const char *>> unrest = {
        {!oracle::same_state(previous, current), "the state kept changing"},
        {faulted, "hardware stayed faulted"},
        {failed_running, "a failed controller stayed active"},
        {trace().hangs_running() > 0, "an injected hang still ran"},
        {trace().calls_running() > 0, "a lifecycle or mode switch call still ran"},
        {catching_up, "an inconsistency or unapplied claim was still waited out"}};
      const bool quiet = std::none_of(
        unrest.begin(), unrest.end(), [](const auto & reason) { return reason.first; });
      if (!quiet)
      {
        quiet_since = Clock::now();
      }
      const bool settled = quiet && Clock::now() - quiet_since >= QUIET_WINDOW;
      if (settled || Clock::now() > deadline)
      {
        if (!settled)
        {
          ++result_.stats.settle_deadline_hits;
          std::string reasons;
          for (const auto & [holds, text] : unrest)
          {
            reasons += holds ? std::string(reasons.empty() ? "" : ", ") + text : "";
          }
          fail(
            check::SETTLE,
            "did not settle in time, " + (reasons.empty() ? "it turned quiet too late" : reasons));
        }
        return {std::move(previous), std::move(current)};
      }
      if (!quiet)
      {
        previous = std::move(current);
      }
    }
  }

  /// Stepped cycles can outpace async hardware, which runs on its own clock.
  void await_io(const Snapshot & since)
  {
    const auto deadline = Clock::now() + IO_DEADLINE;
    while (!oracle::hardware_without_io(since, oracle::take_snapshot(trace())).empty() &&
           Clock::now() < deadline)
    {
      loop_->tick();
    }
  }

  void finish()
  {
    if (!cm_)
    {
      return;
    }
    if (!shut_down_)
    {
      context_ = "shutdown";
      const bool clean = result_.passed();
      const auto result = shut_down({});
      if (!result.accepted)
      {
        fail(result.hung ? check::HANG : check::EXCEPTION, result.error);
      }
      else if (clean)
      {
        report(check::SHUTDOWN, shutdown_findings());
        report_events();
      }
    }
    clear_faults();
    loop_.reset();
    cm_.reset();
  }

  void check_settled(const Window & window)
  {
    report_events();
    for (const auto & error : loop_->take_errors())
    {
      fail(check::EXCEPTION, "cycle() threw: " + error);
    }
    report(check::CONSISTENCY, oracle::check_consistency(window.end));
    report(check::LIVENESS, oracle::check_liveness(window.begin, window.end));
    try
    {
      report(check::STATUS, await_reported([this] { return cm_->reported(); }));
      report(check::ACTIVITY, await_reported([this] { return cm_->activity(); }));
    }
    catch (const std::exception & e)
    {
      fail(check::EXCEPTION, std::string("reporting the state threw: ") + e.what());
    }
  }

  void check_real_time(uint64_t allocations_before, bool check_cycle_time)
  {
    if (rt_allocations() != allocations_before)
    {
      fail(
        check::RT_ALLOCATION,
        std::to_string(rt_allocations() - allocations_before) + " allocations in the control loop");
    }
    const auto longest = loop_->take_longest_cycle();
    if (check_cycle_time)
    {
      result_.stats.longest_cycle = std::max(result_.stats.longest_cycle, longest);
    }
    if (check_cycle_time && longest > LONGEST_CYCLE)
    {
      fail(check::RT_BLOCKING, "a control cycle took " + milliseconds_text(longest));
    }
  }

  /// A report may lag behind the truth, as a fault is finished off RT, but must show it in time.
  template <typename Source>
  oracle::Findings await_reported(const Source & source)
  {
    const auto deadline = Clock::now() + REPORT_DEADLINE;
    while (true)
    {
      auto findings = oracle::check_reported(oracle::take_snapshot(trace()), source());
      if (findings.empty() || Clock::now() > deadline)
      {
        return findings;
      }
      loop_->tick();
    }
  }

  static Names refused_stop(const Snapshot & before, const Snapshot & after)
  {
    Names names;
    for (size_t i = 0; i < rig::controllers().size(); ++i)
    {
      if (before.controllers[i].refused_stop || after.controllers[i].refused_stop)
      {
        names.insert(rig::controllers()[i].name);
      }
    }
    return names;
  }

  void stop_sampling(StatusSampler & sampler)
  {
    sampler.stop();
    if (sampler.error())
    {
      fail(check::EXCEPTION, "reporting the state threw: " + *sampler.error());
    }
  }

  void check_samples(const StatusSampler & sampler, const Names & refused)
  {
    std::vector<uint32_t> controllers;
    for (uint16_t i = 0; i < rig::controllers().size(); ++i)
    {
      controllers.push_back(trace().take_visited_states(i));
    }
    std::vector<uint32_t> hardware;
    for (uint16_t i = 0; i < rig::hardware().size(); ++i)
    {
      hardware.push_back(trace().take_visited_hardware_states(i));
    }
    const auto first_finding = [&](auto source) -> oracle::Findings
    {
      for (const auto & sample : sampler.samples())
      {
        auto findings =
          oracle::check_reported_during(source(sample), controllers, hardware, refused);
        if (!findings.empty())
        {
          return findings;
        }
      }
      return {};
    };
    report(check::STATUS, first_finding([](const StatusSample & s) { return s.reported; }));
    report(check::ACTIVITY, first_finding([](const StatusSample & s) { return s.activity; }));
  }

  void reset_visited()
  {
    for (uint16_t i = 0; i < rig::controllers().size(); ++i)
    {
      trace().take_visited_states(i);
    }
    for (uint16_t i = 0; i < rig::hardware().size(); ++i)
    {
      trace().take_visited_hardware_states(i);
    }
  }

  void report_events()
  {
    for (const auto & summary : summarize(trace().take_events()))
    {
      fail(summary.category, summary.text);
    }
  }

  void expect_accepted(const std::function<bool()> & call, const std::string & what)
  {
    const auto result = operate(call, {});
    if (!result.accepted)
    {
      fail(check::SETUP, what + ": " + result.error);
    }
    else if (!*result.accepted)
    {
      fail(check::SETUP, what + " was rejected");
    }
  }

  void arm(const FaultSpec & fault)
  {
    faults().arm(target_of(fault), fault.hook, fault.fault);
    ++result_.stats.faults_armed[fault_kind(fault)];
    armed_.push_back(fault);
  }

  /// Counts the armed faults a plugin took, then disarms everything.
  void clear_faults()
  {
    for (const auto & fault : armed_)
    {
      if (!faults().pending(target_of(fault), fault.hook))
      {
        ++result_.stats.faults_taken[fault_kind(fault)];
      }
    }
    armed_.clear();
    faults().clear();
  }

  void count_answer(const Step & step, bool accepted)
  {
    const std::string text = to_text(step);
    const auto * switch_step = std::get_if<SwitchStep>(&step);
    auto & answers = result_.stats.answers
                       [switch_step ? std::string("switch ") + to_string(switch_step->strictness)
                                    : text.substr(0, text.find(' '))];
    ++(accepted ? answers.accepted : answers.rejected);
  }

  void report(const char * category, const oracle::Findings & findings)
  {
    for (const auto & finding : findings)
    {
      fail(category, finding);
    }
  }

  /// Whether a finding of this category ends up in the result.
  bool recorded(const std::string & category) const
  {
    const auto group = category.substr(0, category.find(':'));
    return options_.skipped_checks.count(category) == 0 &&
           options_.skipped_checks.count(group) == 0 &&
           !(options_.gates_only && tier_of(category) == Tier::Provisional);
  }

  void fail(const std::string & category, const std::string & message)
  {
    if (recorded(category))
    {
      result_.failures.push_back(
        std::string("[") + to_string(tier_of(category)) + "] [" + category + "] " + context_ +
        ": " + message);
    }
  }

  void log(const std::string & outcome) { result_.log.push_back(context_ + " -> " + outcome); }

  const CmFactory & factory_;
  const RunOptions & options_;
  std::unique_ptr<CmUnderTest> cm_;
  std::unique_ptr<ControlLoop> loop_;
  bool power_cut_ = false;
  /// Whether a controller on unpowered hardware was still updated after its bound, or the loop
  /// stopped cycling before they all stopped.
  std::future<std::optional<std::string>> power_reaction_;
  /// Set by the operation thread when a switch or shutdown returned, read by the stall.
  std::atomic<bool> operation_returned_{false};
  bool shut_down_ = false;
  /// Whether an injected hang still ran when shutdown returned.
  bool hang_outlived_shutdown_ = false;
  /// The faults armed since they were last cleared.
  std::vector<FaultSpec> armed_;
  /// The state before an unchecked fault or lifecycle step, whose effects the next step still
  /// meets.
  std::optional<Snapshot> unchecked_before_;
  /// The controllers of unchecked lifecycle steps.
  Names unchecked_lifecycle_;
  std::string context_;
  RunResult result_;
};
}  // namespace

Tier tier_of(const std::string & category)
{
  static const std::set<std::string> gates = {
    "event:inactive_writer",
    "event:unknown_value",
    "event:misrouted_value",
    "event:limit_exceeded",
    "event:io_while_inactive",
    "event:io_after_fault",
    "event:mode_conflict",
    "event:concurrent_hardware_call",
    "event:concurrent_controller_call",
    "event:update_while_inactive",
    "event:update_after_failure",
    "event:missing_interfaces",
    "event:activated_before_provider",
    "event:provider_stopped_under_user",
    "event:activated_on_inactive_hardware",
    "event:shared_command_interface",
    check::CONSISTENCY,
    check::SHUTDOWN,
    check::POWER_CUT,
    check::EXCEPTION,
    check::HANG,
  };
  return gates.count(category) ? Tier::Gate : Tier::Provisional;
}

const char * to_string(Tier tier) { return tier == Tier::Gate ? "gate" : "provisional"; }

CmConfig default_config()
{
  CmConfig config;
  config.robot_description = rig::urdf();
  config.startup = rig::startup_profile();
  return config;
}

RunResult run_script(const CmFactory & factory, const Script & script, const RunOptions & options)
{
  return Session(factory, options).run(script);
}

RunResult run_rejected_description(
  const CmFactory & factory, const std::string & description, const RunOptions & options)
{
  return Session(factory, options).run_rejected(description);
}

}  // namespace cm_test_suite
