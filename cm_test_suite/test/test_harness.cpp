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
#include <pthread.h>
#include <sched.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <ctime>
#include <fstream>
#include <functional>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "cm_test_suite/rig.hpp"
#include "cm_test_suite/rt_allocations.hpp"
#include "cm_test_suite/runner.hpp"
#include "control_loop.hpp"
#include "controller_tracer.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/system_interface.hpp"
#include "operation.hpp"
#include "plugin_support.hpp"
#include "pluginlib/class_loader.hpp"
#include "status_sampler.hpp"

namespace cm_test_suite
{
namespace
{
using std::chrono::milliseconds;

/// Accepts everything and does nothing, enough to drive the harness machinery.
class IdleCm : public CmUnderTest
{
public:
  void cycle() override {}
  bool load(const std::string &) override { return true; }
  bool configure(const std::string &) override { return true; }
  bool cleanup(const std::string &) override { return true; }
  bool unload(const std::string &) override { return true; }
  bool switch_controllers(
    const std::vector<std::string> &, const std::vector<std::string> &, Strictness) override
  {
    std::this_thread::sleep_for(milliseconds(20));
    return true;
  }
  bool recover_hardware(const std::string &) override { return true; }
  ReportedState reported() override { return {}; }
  ReportedState activity() override { return {}; }
  void shutdown() override {}
};

/// Whether an event of this kind, concerning writer if given, was reported since the last take.
bool took_event(EventKind kind, std::optional<uint16_t> writer = std::nullopt)
{
  const auto events = trace().take_events();
  return std::any_of(
    events.begin(), events.end(),
    [&](const Event & e) { return e.kind == kind && (!writer || e.writer == *writer); });
}

/// Whether a failure contains the text, such as a category and phase.
bool failed_with(const RunResult & result, const std::string & text)
{
  return std::any_of(
    result.failures.begin(), result.failures.end(),
    [&](const std::string & failure) { return failure.find(text) != std::string::npos; });
}

/// The rig's arm loaded as a plugin and initialized from the rig description.
class ArmPlugin
{
public:
  ArmPlugin()
  {
    params_.hardware_info =
      hardware_interface::parse_control_resources_from_urdf(rig::urdf()).at(0);
    params_.clock = std::make_shared<rclcpp::Clock>();
    if (arm_->init(params_) != CallbackReturn::SUCCESS)
    {
      throw std::runtime_error("The rig arm did not initialize");
    }
    commands_ = arm_->on_export_command_interfaces();
  }

  hardware_interface::SystemInterface * operator->() { return arm_.get(); }

  hardware_interface::CommandInterface::SharedPtr command(const std::string & name) const
  {
    return *std::find_if(
      commands_.begin(), commands_.end(), [&](const auto & c) { return c->get_name() == name; });
  }

  hardware_interface::return_type read()
  {
    return arm_->read(params_.clock->now(), rclcpp::Duration::from_seconds(0.01));
  }

  hardware_interface::return_type write()
  {
    return arm_->write(params_.clock->now(), rclcpp::Duration::from_seconds(0.01));
  }

private:
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader_{
    "hardware_interface", "hardware_interface::SystemInterface"};
  pluginlib::UniquePtr<hardware_interface::SystemInterface> arm_ =
    loader_.createUniqueInstance(rig::TRACE_SYSTEM);
  hardware_interface::HardwareComponentParams params_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> commands_;
};
}  // namespace

TEST(Hang, an_activation_hangs_counted_and_then_gives_up)
{
  trace().reset();
  faults().clear();
  const Target jsb{TargetKind::Controller, 0};
  faults().arm(jsb, Hook::Activate, Fault::Hang);
  std::optional<CallbackReturn> result;
  int hangs_after_return = 0;
  std::thread callback(
    [&]
    {
      std::atomic<bool> busy{false};
      CallGuard call(busy, EventKind::ConcurrentControllerCall, jsb);
      result = take_lifecycle_fault(jsb, Hook::Activate, call);
      hangs_after_return = trace().hangs_running();
    });
  std::this_thread::sleep_for(milliseconds(100));
  EXPECT_EQ(1, trace().hangs_running());
  callback.join();
  EXPECT_EQ(1, hangs_after_return) << "a hang must count until its callback returns";
  EXPECT_EQ(0, trace().hangs_running());
  EXPECT_EQ(CallbackReturn::FAILURE, result);
}

TEST(RtAllocations, an_injected_throw_allocates_nothing)
{
  if (!rt_allocation_counting_available())
  {
    GTEST_SKIP() << "Sanitizer builds do not count allocations";
  }
  const uint64_t before = rt_allocations();
  {
    const RtScope rt;
    try
    {
      throw InjectedFault(Hook::Read);
    }
    catch (const std::exception & e)
    {
      EXPECT_STREQ("Injected fault in read", e.what());
    }
  }
  EXPECT_EQ(before, rt_allocations());
}

TEST(Order, a_provider_may_stop_under_a_user_hanging_in_a_callback)
{
  trace().reset();
  faults().clear();
  const uint16_t whole_body = *rig::controller_index("whole_body");
  trace().set_controller_state(whole_body, Lifecycle::Active);
  ControllerTracer arm_ik("arm_ik");
  const auto stopped_under_user = []
  {
    const auto events = trace().take_events();
    return std::any_of(
      events.begin(), events.end(),
      [](const Event & e) { return e.kind == EventKind::ProviderStoppedUnderUser; });
  };
  trace().begin_hang({TargetKind::Controller, whole_body});
  arm_ik.deactivate();
  EXPECT_FALSE(stopped_under_user());
  trace().end_hang({TargetKind::Controller, whole_body});
  arm_ik.deactivate();
  EXPECT_TRUE(stopped_under_user());
}

TEST(Order, an_activation_beside_an_active_claimer_of_its_commands_or_joints_is_reported)
{
  faults().clear();
  const auto shared = [](const char * name, const char * active)
  {
    trace().reset();
    const uint16_t other = *rig::controller_index(active);
    trace().set_controller_state(other, Lifecycle::Active);
    ControllerTracer controller(name);
    controller.activate({}, {});
    return took_event(EventKind::SharedCommandInterface, other);
  };
  EXPECT_TRUE(shared("arm_pos", "arm_vel")) << "another mode of the same joints";
  EXPECT_TRUE(shared("flipper_vel", "flipper_pos")) << "another mode of the same joint";
  EXPECT_TRUE(shared("nav", "whole_body")) << "the same references";
  EXPECT_FALSE(shared("arm_pos", "flipper_vel"));
  EXPECT_FALSE(shared("wrist", "arm_ik")) << "two writers of different references";
}

TEST(Order, a_hanging_claimer_may_share_with_an_activation_but_a_failed_one_may_not)
{
  trace().reset();
  faults().clear();
  const uint16_t arm_vel = *rig::controller_index("arm_vel");
  trace().set_controller_state(arm_vel, Lifecycle::Active);
  ControllerTracer arm_pos("arm_pos");
  trace().begin_hang({TargetKind::Controller, arm_vel});
  arm_pos.activate({}, {});
  EXPECT_FALSE(took_event(EventKind::SharedCommandInterface));
  trace().end_hang({TargetKind::Controller, arm_vel});
  trace().set_controller_failed(arm_vel);
  arm_pos.activate({}, {});
  EXPECT_TRUE(took_event(EventKind::SharedCommandInterface))
    << "a failed controller may still write its commands in on_deactivate";
}

TEST(Liveness, a_controller_counts_its_updates)
{
  trace().reset();
  faults().clear();
  ControllerTracer monitor("monitor");
  const uint16_t index = *rig::controller_index("monitor");
  monitor.begin_update({}, {});
  monitor.begin_update({}, {});
  EXPECT_EQ(2u, trace().controller(index).updates);
}

TEST(Order, an_activation_above_an_inactive_or_unchained_provider_is_reported)
{
  trace().reset();
  faults().clear();
  const uint16_t arm_vel = *rig::controller_index("arm_vel");
  ControllerTracer arm_ik("arm_ik");
  const auto activation_reports = [&](EventKind kind)
  {
    arm_ik.activate({}, {});
    return took_event(kind, arm_vel);
  };
  EXPECT_TRUE(activation_reports(EventKind::ActivatedBeforeProvider));
  trace().set_controller_state(arm_vel, Lifecycle::Active);
  EXPECT_FALSE(activation_reports(EventKind::ActivatedBeforeProvider));
  EXPECT_TRUE(activation_reports(EventKind::ActivatedOnUnchainedProvider));
  trace().set_controller_chained(arm_vel, true);
  EXPECT_FALSE(activation_reports(EventKind::ActivatedOnUnchainedProvider));
}

TEST(Order, an_activation_on_unavailable_hardware_is_reported)
{
  trace().reset();
  faults().clear();
  const auto reported = [](const char * name)
  {
    ControllerTracer controller(name);
    controller.activate({}, {});
    return took_event(EventKind::ActivatedOnInactiveHardware);
  };
  const uint16_t arm = *rig::hardware_index("arm");
  const uint16_t imu = *rig::hardware_index("imu");
  trace().set_hardware_state(arm, Lifecycle::Inactive);
  EXPECT_TRUE(reported("arm_pos")) << "commanded hardware must be active";
  trace().set_hardware_state(arm, Lifecycle::Active);
  EXPECT_FALSE(reported("arm_pos"));
  trace().set_hardware_state(imu, Lifecycle::Inactive);
  EXPECT_FALSE(reported("odom_pub")) << "hardware that is only read may be inactive";
  trace().set_hardware_state(imu, Lifecycle::Unconfigured);
  EXPECT_TRUE(reported("odom_pub"));
}

TEST(Update, an_update_while_inactive_or_after_a_failure_is_reported)
{
  trace().reset();
  faults().clear();
  ControllerTracer monitor("monitor");
  monitor.configure();
  monitor.begin_update({}, {});
  EXPECT_TRUE(took_event(EventKind::UpdateWhileInactive));
  monitor.activate({}, {});
  monitor.begin_update({}, {});
  EXPECT_FALSE(took_event(EventKind::UpdateWhileInactive));
  trace().set_controller_failed(*rig::controller_index("monitor"));
  monitor.begin_update({}, {});
  EXPECT_TRUE(took_event(EventKind::UpdateAfterFailure));
  monitor.deactivate();
  monitor.activate({}, {});
  monitor.begin_update({}, {});
  EXPECT_FALSE(took_event(EventKind::UpdateAfterFailure)) << "a restart ends the failure";
}

TEST(Concurrency, a_call_into_a_controller_busy_with_another_is_reported)
{
  trace().reset();
  ControllerTracer monitor("monitor");
  {
    const auto first = monitor.guard();
    EXPECT_FALSE(took_event(EventKind::ConcurrentControllerCall));
    const auto second = monitor.guard();
    EXPECT_TRUE(took_event(EventKind::ConcurrentControllerCall));
  }
  const auto after_return = monitor.guard();
  EXPECT_FALSE(took_event(EventKind::ConcurrentControllerCall));
}

TEST(Quiet, a_quiet_activation_skips_the_first_updates_only)
{
  trace().reset();
  faults().clear();
  ControllerTracer flipper_pos("flipper_pos");
  const Target target{TargetKind::Controller, *rig::controller_index("flipper_pos")};
  faults().arm(target, Hook::Activate, Fault::Quiet);
  flipper_pos.activate({}, {});
  EXPECT_EQ(Lifecycle::Active, trace().controller(target.index).state);
  for (int i = 0; i < QUIET_UPDATES; ++i)
  {
    EXPECT_TRUE(flipper_pos.quiet_update()) << "update " << i;
  }
  EXPECT_FALSE(flipper_pos.quiet_update());
  flipper_pos.deactivate();
  flipper_pos.activate({}, {});
  EXPECT_FALSE(flipper_pos.quiet_update()) << "only the quiet activation skips updates";
}

TEST(Late, a_late_callback_takes_nearly_the_switch_timeout_counts_as_hanging_and_fires)
{
  trace().reset();
  faults().clear();
  const Target jsb{TargetKind::Controller, 0};
  faults().arm(jsb, Hook::Deactivate, Fault::Late);
  const auto start = ControlLoop::Clock::now();
  std::thread callback(
    [&]
    {
      std::atomic<bool> busy{false};
      CallGuard call(busy, EventKind::ConcurrentControllerCall, jsb);
      EXPECT_EQ(std::nullopt, take_lifecycle_fault(jsb, Hook::Deactivate, call));
    });
  std::this_thread::sleep_for(milliseconds(100));
  EXPECT_TRUE(trace().controller_hanging(jsb.index))
    << "the controller manager may give up on it, so it is out of the running set";
  callback.join();
  EXPECT_GE(ControlLoop::Clock::now() - start, LATE);
  EXPECT_EQ(0, trace().hangs_running());
  EXPECT_EQ(1u, faults().take_fired().size());
}

TEST(BestEffort, a_reader_may_skip_the_states_of_hardware_in_its_fault_handling_or_hanging)
{
  trace().reset();
  faults().clear();
  for (uint16_t i = 0; i < rig::hardware().size(); ++i)
  {
    trace().set_hardware_state(i, Lifecycle::Active);
  }
  trace().set_hardware_faulted(*rig::hardware_index("arm"), true);
  // Before ros2_control 6.12 a loan only references its interface, so the test owns them
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> owned;
  const auto loans = [&](const std::vector<std::string> & names)
  {
    ControllerTracer::StateInterfaces loaned;
    for (const auto & name : names)
    {
      const auto slash = name.find('/');
      owned.push_back(
        std::make_shared<const hardware_interface::StateInterface>(
          name.substr(0, slash), name.substr(slash + 1)));
      loaned.emplace_back(owned.back());
    }
    return loaned;
  };
  const auto missing = [&](const std::vector<std::string> & names)
  {
    trace().take_events();
    ControllerTracer jsb("jsb");
    jsb.activate({}, loans(names));
    const auto events = trace().take_events();
    return std::any_of(
      events.begin(), events.end(),
      [](const Event & e) { return e.kind == EventKind::MissingInterfaces; });
  };
  EXPECT_FALSE(missing({"wl/position", "wr/position", "f1/position"}));
  EXPECT_FALSE(missing(
    {"a1/position", "a2/position", "a3/position", "wl/position", "wr/position", "f1/position"}));
  EXPECT_TRUE(missing({"a1/position", "a2/position", "a3/position", "wl/position", "f1/position"}))
    << "wr is not faulted, so it must be claimed";
  // The controller manager may give up on base while its read hangs (fuzz seed 725, all async)
  const Target base{TargetKind::Hardware, *rig::hardware_index("base")};
  trace().begin_hang(base);
  EXPECT_FALSE(missing({"f1/position"}));
  trace().end_hang(base);
  EXPECT_TRUE(missing({"f1/position"})) << "base no longer hangs, so it must be claimed";
}

TEST(Operation, a_harness_stall_does_not_count_as_operation_time)
{
  IdleCm cm;
  ControlLoop loop(cm, Mode::Stepped);
  const auto start = ControlLoop::Clock::now();
  const auto result = run_operation(
    loop, [&] { return cm.switch_controllers({}, {}, Strictness::Strict); },
    {{1, [&] { loop.stall(milliseconds(300)); }}}, std::chrono::seconds(5));
  EXPECT_GE(ControlLoop::Clock::now() - start, milliseconds(300));
  EXPECT_LT(result.duration, milliseconds(200));
  EXPECT_EQ(true, result.accepted);
}

/// Its cycle runs the given work.
class WorkingCm : public IdleCm
{
public:
  explicit WorkingCm(std::function<void()> work) : work_(std::move(work)) {}
  void cycle() override { work_(); }

private:
  const std::function<void()> work_;
};

TEST(CycleTime, a_cycle_that_sleeps_counts_as_long)
{
  WorkingCm cm([] { std::this_thread::sleep_for(milliseconds(30)); });
  ControlLoop loop(cm, Mode::Stepped);
  loop.tick();
  EXPECT_GE(loop.take_longest_cycle(), milliseconds(30));
}

std::chrono::nanoseconds thread_cpu_time()
{
  timespec now{};
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &now);
  return std::chrono::seconds(now.tv_sec) + std::chrono::nanoseconds(now.tv_nsec);
}

void pin_to(int cpu)
{
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(cpu, &set);
  ASSERT_EQ(0, pthread_setaffinity_np(pthread_self(), sizeof(set), &set));
}

/// Runs work on a thread that shares its core with three busy threads.
void run_on_a_busy_core(const std::function<void()> & work)
{
  const int cpu = sched_getcpu();
  constexpr int COMPETITORS = 3;
  std::atomic<bool> competing{true};
  std::atomic<int> pinned{0};
  std::vector<std::thread> competitors;
  for (int i = 0; i < COMPETITORS; ++i)
  {
    competitors.emplace_back(
      [&]
      {
        pin_to(cpu);
        ++pinned;
        while (competing)
        {
        }
      });
  }
  std::thread working(
    [&]
    {
      pin_to(cpu);
      // A competitor that starts late leaves the work alone on the core
      while (pinned < COMPETITORS)
      {
        std::this_thread::yield();
      }
      work();
    });
  working.join();
  competing = false;
  for (auto & competitor : competitors)
  {
    competitor.join();
  }
}

/// Spins for the given time on the calling thread's own clock and returns the wall time it took.
ControlLoop::Clock::duration spin(milliseconds cpu_time)
{
  const auto start = ControlLoop::Clock::now();
  const auto cpu_start = thread_cpu_time();
  while (thread_cpu_time() - cpu_start < cpu_time)
  {
  }
  return ControlLoop::Clock::now() - start;
}

TEST(CycleTime, a_cycle_preempted_by_busy_threads_on_its_core_counts_its_own_time_only)
{
  constexpr milliseconds SPIN{10};
  ControlLoop::Clock::duration wall{};
  ControlLoop::Clock::duration longest{};
  run_on_a_busy_core(
    [&]
    {
      WorkingCm cm([&] { wall = spin(SPIN); });
      ControlLoop loop(cm, Mode::Stepped);
      loop.tick();
      longest = loop.take_longest_cycle();
    });
  ASSERT_GT(wall, 2 * SPIN) << "the competitors must preempt the cycle";
  EXPECT_GT(longest, SPIN / 2) << "spinning is the cycle's own time";
  EXPECT_LT(longest, SPIN + SPIN / 2);
}

// glibc's lock of its stream list, which every stream open and close takes
extern "C" void _IO_list_lock();
extern "C" void _IO_list_unlock();

TEST(PreemptedTime, does_not_wait_for_a_thread_holding_the_stdio_lock)
{
  std::atomic<bool> returned{false};
  _IO_list_lock();
  std::thread reader(
    [&]
    {
      preempted_time();
      returned = true;
    });
  const auto deadline = ControlLoop::Clock::now() + std::chrono::seconds(1);
  while (!returned && ControlLoop::Clock::now() < deadline)
  {
    std::this_thread::sleep_for(milliseconds(1));
  }
  const bool in_time = returned;
  _IO_list_unlock();
  reader.join();
  EXPECT_TRUE(in_time) << "a preempted lock holder would delay every age check";
}

/// A token of a running writer, older than MAX_COMMAND_AGE_CYCLES.
uint64_t old_token_by_cycles()
{
  trace().reset();
  const auto arm_vel = *rig::controller_index("arm_vel");
  trace().set_controller_state(arm_vel, Lifecycle::Active);
  const uint64_t token = trace().mint(arm_vel, *rig::interface_id("a3/velocity"), 0);
  for (uint64_t i = 0; i <= MAX_COMMAND_AGE_CYCLES; ++i)
  {
    trace().begin_cycle();
  }
  return token;
}

std::optional<EventKind> async_audit(uint64_t token, std::chrono::nanoseconds preempted)
{
  const auto event = trace().audit(
    token, *rig::interface_id("a3/velocity"), {TargetKind::Hardware, 0}, Applier::AsyncThread,
    preempted);
  return event ? std::optional<EventKind>(event->kind) : std::nullopt;
}

TEST(StaleValue, a_value_old_because_its_async_applier_slept_is_stale)
{
  const uint64_t token = old_token_by_cycles();
  // Preemption before the mint is no part of the age
  const auto preempted_before = preempted_time();
  std::this_thread::sleep_for(MAX_COMMAND_AGE + milliseconds(2));
  EXPECT_EQ(EventKind::StaleValue, async_audit(token, preempted_time() - preempted_before));
}

TEST(StaleValue, a_value_old_only_because_its_async_applier_was_preempted_is_fresh)
{
  ControlLoop::Clock::duration wall{};
  std::optional<EventKind> on_wall_clock;
  std::optional<EventKind> without_preemption;
  run_on_a_busy_core(
    [&]
    {
      const auto preempted_before = preempted_time();
      const uint64_t token = old_token_by_cycles();
      // Longer than a scheduler slice, so the competitors run in between, but younger than the
      // age bound on the applier's own clock
      wall = spin(std::chrono::duration_cast<milliseconds>(MAX_COMMAND_AGE * 4 / 5));
      on_wall_clock = async_audit(token, {});
      without_preemption = async_audit(token, preempted_time() - preempted_before);
    });
  ASSERT_GT(wall, MAX_COMMAND_AGE + milliseconds(2)) << "the competitors must preempt the applier";
  EXPECT_EQ(EventKind::StaleValue, on_wall_clock);
  EXPECT_EQ(std::nullopt, without_preemption);
}

/// The async audit of a token minted in the first cycle of a stepped loop, which then runs `work`,
/// once it is older than MAX_COMMAND_AGE_CYCLES.
std::optional<EventKind> audit_after_a_working_minting_cycle(const std::function<void()> & work)
{
  trace().reset();
  const auto arm_vel = *rig::controller_index("arm_vel");
  trace().set_controller_state(arm_vel, Lifecycle::Active);
  std::optional<uint64_t> token;
  std::chrono::nanoseconds preempted_after_work{};
  WorkingCm cm(
    [&]
    {
      if (!token)
      {
        token = trace().mint(arm_vel, *rig::interface_id("a3/velocity"), 0);
        work();
        preempted_after_work = preempted_time();
      }
    });
  ControlLoop loop(cm, Mode::Stepped);
  loop.run_cycles(MAX_COMMAND_AGE_CYCLES + 2);
  // The loop thread applies it, so its wait after the handoff is the applier's
  return async_audit(*token, preempted_time() - preempted_after_work);
}

TEST(
  StaleValue, a_value_old_only_because_the_control_loop_was_preempted_before_its_handoff_is_fresh)
{
  ControlLoop::Clock::duration wall{};
  std::optional<EventKind> event;
  run_on_a_busy_core(
    [&]
    {
      event = audit_after_a_working_minting_cycle(
        [&] { wall = spin(std::chrono::duration_cast<milliseconds>(MAX_COMMAND_AGE * 4 / 5)); });
    });
  ASSERT_GT(wall, MAX_COMMAND_AGE + milliseconds(2)) << "the competitors must preempt the loop";
  EXPECT_EQ(std::nullopt, event);
}

TEST(StaleValue, a_value_old_because_the_control_loop_worked_before_its_handoff_is_stale)
{
  EXPECT_EQ(
    EventKind::StaleValue,
    audit_after_a_working_minting_cycle([] { spin(MAX_COMMAND_AGE + milliseconds(1)); }));
}

/// Sets each chained mode to whether an active controller writes it, as a CM does.
void set_chained_modes()
{
  for (const auto & controller : rig::controllers())
  {
    const rig::Names writers = rig::reference_writers(controller.name);
    trace().set_controller_chained(
      *rig::controller_index(controller.name),
      std::any_of(
        writers.begin(), writers.end(), [](const std::string & writer)
        { return trace().controller(*rig::controller_index(writer)).state == Lifecycle::Active; }));
  }
}

/// Brings all hardware up and nothing else. Async hardware is read and written on its own clock,
/// every 2 ms, like a timer driven async thread. Every active controller whose update did not fail
/// is updated each cycle.
class SlowAsyncCm : public IdleCm
{
public:
  SlowAsyncCm()
  {
    for (uint16_t i = 0; i < rig::hardware().size(); ++i)
    {
      trace().set_hardware_state(i, Lifecycle::Active);
    }
  }

  void cycle() override
  {
    const bool async_due = ControlLoop::Clock::now() - last_async_io_ > milliseconds(2);
    for (uint16_t i = 0; i < rig::hardware().size(); ++i)
    {
      if (!rig::hardware()[i].is_async || async_due)
      {
        trace().count_read(i);
        trace().count_write(i);
      }
    }
    if (async_due)
    {
      last_async_io_ = ControlLoop::Clock::now();
    }
    for (uint16_t i = 0; i < rig::controllers().size(); ++i)
    {
      const auto truth = trace().controller(i);
      if (truth.state == Lifecycle::Active && truth.failed_epoch != truth.epoch && updates(i))
      {
        trace().count_update(i);
      }
    }
  }

  void shutdown() override
  {
    for (uint16_t i = 0; i < rig::hardware().size(); ++i)
    {
      trace().set_hardware_state(i, Lifecycle::Inactive);
    }
  }

protected:
  /// Whether an active controller is updated this cycle.
  virtual bool updates(uint16_t) const { return true; }

private:
  ControlLoop::Clock::time_point last_async_io_ = ControlLoop::Clock::now();
};

/// Reports every component inactive until `lag` after the first report was asked for, then the
/// truth. The truth is up from its creation, so the lag starts at the settled check on any build.
class LaggingCm : public SlowAsyncCm
{
public:
  explicit LaggingCm(milliseconds lag) : lag_(lag) {}

  ReportedState reported() override
  {
    bool caught_up = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto now = ControlLoop::Clock::now();
      if (!first_read_)
      {
        first_read_ = now;
      }
      caught_up = now >= *first_read_ + lag_;
    }
    ReportedState state;
    for (uint16_t i = 0; i < rig::hardware().size(); ++i)
    {
      state.hardware[rig::hardware()[i].name] =
        caught_up ? trace().hardware(i).state : Lifecycle::Inactive;
    }
    return state;
  }
  ReportedState activity() override { return reported(); }

private:
  const milliseconds lag_;
  std::mutex mutex_;
  std::optional<ControlLoop::Clock::time_point> first_read_;
};

TEST(Status, a_report_may_catch_up_with_the_settled_truth_within_500_ms)
{
  RunOptions options;
  options.config = default_config();
  const auto status_failed = [&](milliseconds lag)
  {
    const auto result =
      run_script([lag](const CmConfig &) { return std::make_unique<LaggingCm>(lag); }, {}, options);
    return std::any_of(
      result.failures.begin(), result.failures.end(),
      [](const std::string & failure) { return failure.find("[status]") != std::string::npos; });
  };
  EXPECT_FALSE(status_failed(milliseconds(300)));
  EXPECT_TRUE(status_failed(milliseconds(5000))) << "a report that never catches up fails";
}

TEST(Settle, waits_for_async_hardware_on_its_own_clock)
{
  RunOptions options;
  options.config = default_config();
  const auto result =
    run_script([](const CmConfig &) { return std::make_unique<SlowAsyncCm>(); }, {}, options);
  for (const auto & failure : result.failures)
  {
    EXPECT_EQ(std::string::npos, failure.find("[liveness]")) << failure;
  }
  // The fake reports nothing, so the status check after liveness fails
  EXPECT_TRUE(failed_with(result, "[status] setup"));
}

/// Loads and configures at once, but finishes an activation on another thread a few
/// milliseconds after the switch returned, like a controller manager that works off RT.
class LateActivationCm : public SlowAsyncCm
{
public:
  ~LateActivationCm() override
  {
    if (worker_.joinable())
    {
      worker_.join();
    }
  }

  bool load(const std::string & name) override
  {
    trace().set_controller_state(*rig::controller_index(name), Lifecycle::Unconfigured);
    return true;
  }

  bool configure(const std::string & name) override
  {
    trace().set_controller_state(*rig::controller_index(name), Lifecycle::Inactive);
    return true;
  }

  bool switch_controllers(
    const std::vector<std::string> & activate, const std::vector<std::string> &,
    Strictness) override
  {
    worker_ = std::thread(
      [activate]
      {
        std::this_thread::sleep_for(milliseconds(5));
        for (const auto & name : activate)
        {
          trace().set_controller_state(*rig::controller_index(name), Lifecycle::Active);
        }
        set_chained_modes();
      });
    return true;
  }

private:
  std::thread worker_;
};

TEST(Settle, waits_for_a_change_finished_off_the_control_loop)
{
  RunOptions options;
  options.config = default_config();
  const auto result =
    run_script([](const CmConfig &) { return std::make_unique<LateActivationCm>(); }, {}, options);
  for (const auto & failure : result.failures)
  {
    EXPECT_EQ(std::string::npos, failure.find("[setup]")) << failure;
  }
  // The fake reports nothing, so the status check after the setup check fails
  EXPECT_TRUE(failed_with(result, "[status] setup"));
}

/// Activates at once, then a controller's update fails and the controller manager stops it off the
/// control loop, later than the quiet window.
class SlowFaultReactionCm : public LateActivationCm
{
public:
  ~SlowFaultReactionCm() override
  {
    if (reaction_.joinable())
    {
      reaction_.join();
    }
  }

  bool switch_controllers(
    const std::vector<std::string> & activate, const std::vector<std::string> &,
    Strictness) override
  {
    for (const auto & name : activate)
    {
      trace().set_controller_state(*rig::controller_index(name), Lifecycle::Active);
    }
    set_chained_modes();
    const uint16_t failed = *rig::controller_index(activate.front());
    trace().set_controller_failed(failed);
    reaction_ = std::thread(
      [failed]
      {
        std::this_thread::sleep_for(milliseconds(60));
        trace().set_controller_state(failed, Lifecycle::Inactive);
        set_chained_modes();
      });
    return true;
  }

private:
  std::thread reaction_;
};

TEST(Settle, waits_while_a_failed_controller_is_still_active)
{
  RunOptions options;
  options.config = default_config();
  const auto result = run_script(
    [](const CmConfig &) { return std::make_unique<SlowFaultReactionCm>(); }, {}, options);
  for (const auto & failure : result.failures)
  {
    EXPECT_EQ(std::string::npos, failure.find("after its update failed")) << failure;
  }
  // The failed controller is stopped when the setup is checked, so it is missing from it
  EXPECT_TRUE(failed_with(result, "[setup] setup: active [arm_vel base_vel"));
}

/// Starts with the chained modes of the rig, and stops arm_vel `gap` after its writers arm_ik and
/// wrist, as a CM that deactivates one controller per step, or never.
class ChainTeardownCm : public LateActivationCm
{
public:
  explicit ChainTeardownCm(std::optional<milliseconds> gap) : gap_(gap) {}

  ~ChainTeardownCm() override
  {
    if (teardown_.joinable())
    {
      teardown_.join();
    }
  }

  bool switch_controllers(
    const std::vector<std::string> & activate, const std::vector<std::string> & deactivate,
    Strictness) override
  {
    for (const auto & name : activate)
    {
      trace().set_controller_state(*rig::controller_index(name), Lifecycle::Active);
    }
    set_chained_modes();
    const uint16_t arm_vel = *rig::controller_index("arm_vel");
    for (const auto & name : deactivate)
    {
      if (name != "arm_vel")
      {
        trace().set_controller_state(*rig::controller_index(name), Lifecycle::Inactive);
      }
      else if (gap_)
      {
        if (teardown_.joinable())
        {
          throw std::logic_error("ChainTeardownCm stops arm_vel only once");
        }
        teardown_ = std::thread(
          [arm_vel, gap = *gap_]
          {
            std::this_thread::sleep_for(gap);
            trace().set_controller_chained(arm_vel, false);
            trace().set_controller_state(arm_vel, Lifecycle::Inactive);
          });
      }
    }
    return true;
  }

private:
  const std::optional<milliseconds> gap_;
  std::thread teardown_;
};

TEST(Settle, waits_out_a_provider_chained_between_two_teardown_steps)
{
  RunOptions options;
  options.config = default_config();
  // The fake neither applies interfaces nor reports a state
  options.skipped_checks = {check::LIVENESS, check::STATUS, check::ACTIVITY};
  const auto consistency_failed = [&](std::optional<milliseconds> gap)
  {
    const auto result = run_script(
      [gap](const CmConfig &) { return std::make_unique<ChainTeardownCm>(gap); },
      parse_script("switch strict -arm_ik -wrist -arm_vel\n"), options);
    return std::any_of(
      result.failures.begin(), result.failures.end(), [](const std::string & failure)
      { return failure.find("[consistency]") != std::string::npos; });
  };
  EXPECT_FALSE(consistency_failed(milliseconds(30)));
  EXPECT_TRUE(consistency_failed(milliseconds(1500))) << "longer than one supervisor step";
  EXPECT_TRUE(consistency_failed(std::nullopt)) << "a provider that stays chained is a gate";
}

TEST(Settle, does_not_wait_for_a_consistency_that_is_not_checked)
{
  RunOptions options;
  options.config = default_config();
  options.skipped_checks = {check::LIVENESS, check::STATUS, check::ACTIVITY, check::CONSISTENCY};
  const auto start = ControlLoop::Clock::now();
  run_script(
    [](const CmConfig &) { return std::make_unique<ChainTeardownCm>(std::nullopt); },
    parse_script("switch strict -arm_ik -wrist -arm_vel\n"), options);
  // About 2 s, waiting on the lasting inconsistency adds the settle deadline of 3.5 s
  EXPECT_LT(ControlLoop::Clock::now() - start, milliseconds(3500));
}

/// Stops a controller whose update failed through its on_deactivate on another thread, and records
/// how many injected hangs ran when a switch arrived.
class FaultHandlingCm : public LateActivationCm
{
public:
  explicit FaultHandlingCm(std::atomic<int> & hangs_at_switch) : hangs_at_switch_(hangs_at_switch)
  {
  }

  ~FaultHandlingCm() override
  {
    if (handler_.joinable())
    {
      handler_.join();
    }
  }

  void cycle() override
  {
    SlowAsyncCm::cycle();
    const Target wrist{TargetKind::Controller, *rig::controller_index("wrist")};
    if (faults().take(wrist, Hook::Update) != Fault::None)
    {
      handler_ = std::thread(
        [wrist]
        {
          std::atomic<bool> busy{false};
          CallGuard call(busy, EventKind::ConcurrentControllerCall, wrist);
          take_lifecycle_fault(wrist, Hook::Deactivate, call);
          trace().set_controller_state(wrist.index, Lifecycle::Inactive);
          set_chained_modes();
        });
    }
  }

  bool switch_controllers(
    const std::vector<std::string> & activate, const std::vector<std::string> &,
    Strictness) override
  {
    hangs_at_switch_ = std::max(hangs_at_switch_.load(), trace().hangs_running());
    for (const auto & name : activate)
    {
      trace().set_controller_state(*rig::controller_index(name), Lifecycle::Active);
    }
    set_chained_modes();
    return true;
  }

private:
  std::atomic<int> & hangs_at_switch_;
  std::thread handler_;
};

TEST(FaultStep, without_wait_the_next_switch_meets_the_hung_fault_handling)
{
  RunOptions options;
  options.config = default_config();
  // The fake controller manager fails most checks, and the run must go on past them
  options.skipped_checks = {check::SETUP,    check::CONSISTENCY, check::LIVENESS, check::STATUS,
                            check::ACTIVITY, check::OUTCOME,     check::EVENT};
  std::atomic<int> hangs_at_switch{0};
  const auto result = run_script(
    [&](const CmConfig &) { return std::make_unique<FaultHandlingCm>(hangs_at_switch); },
    parse_script(
      "fault controller:wrist:update:error fault=controller:wrist:deactivate:hang nowait\n"
      "switch strict +flipper_vel -flipper_pos\n"),
    options);
  EXPECT_EQ(1, hangs_at_switch.load());
  EXPECT_EQ(2u, result.log.size()) << "both steps ran";
}

TEST(Stats, count_answers_and_armed_against_taken_faults)
{
  RunOptions options;
  options.config = default_config();
  // The fake fails most checks, its fault handler thread allocates in a cycle
  options.skipped_checks = {check::SETUP,  check::CONSISTENCY,  check::LIVENESS,
                            check::STATUS, check::ACTIVITY,     check::OUTCOME,
                            check::EVENT,  check::RT_ALLOCATION};
  std::atomic<int> hangs_at_switch{0};
  // The fake takes only the update fault of wrist
  const auto result = run_script(
    [&](const CmConfig &) { return std::make_unique<FaultHandlingCm>(hangs_at_switch); },
    parse_script(
      "fault controller:wrist:update:error\n"
      "fault controller:nav:update:throw\n"
      "switch strict +flipper_vel -flipper_pos\n"),
    options);
  const auto & stats = result.stats;
  ASSERT_EQ(3u, result.log.size()) << testing::PrintToString(result.failures);
  EXPECT_EQ(2u, stats.answers.at("fault").accepted);
  EXPECT_EQ(1u, stats.answers.at("switch strict").accepted);
  EXPECT_EQ(0u, stats.answers.at("switch strict").rejected);
  EXPECT_EQ(
    (std::map<std::string, size_t>{{"update:error", 1}, {"update:throw", 1}}), stats.faults_armed);
  EXPECT_EQ((std::map<std::string, size_t>{{"update:error", 1}}), stats.faults_taken);
  EXPECT_EQ(0u, stats.unpowered_steps);
  EXPECT_GT(stats.longest_cycle.count(), 0);
}

/// Records when the cycles saw slow async I/O.
class PacedCm : public IdleCm
{
public:
  using TimePoint = ControlLoop::Clock::time_point;

  PacedCm(std::optional<TimePoint> & first_slow, TimePoint & last_slow)
  : first_slow_(first_slow), last_slow_(last_slow)
  {
  }

  void cycle() override
  {
    const auto start = ControlLoop::Clock::now();
    trace().pace_async_io(false);
    const auto now = ControlLoop::Clock::now();
    if (now - start >= SLOW_IO)
    {
      first_slow_ = first_slow_.value_or(now);
      last_slow_ = now;
    }
  }

private:
  std::optional<TimePoint> & first_slow_;
  TimePoint & last_slow_;
};

TEST(SlowIo, async_hardware_is_slow_for_longer_than_the_hang_rule_while_the_loop_cycles)
{
  RunOptions options;
  options.config = default_config();
  options.skipped_checks = {check::SETUP,    check::CONSISTENCY, check::LIVENESS, check::STATUS,
                            check::ACTIVITY, check::OUTCOME,     check::EVENT};
  std::optional<PacedCm::TimePoint> first_slow;
  PacedCm::TimePoint last_slow;
  const auto result = run_script(
    [&](const CmConfig &) { return std::make_unique<PacedCm>(first_slow, last_slow); },
    parse_script("slowio\nswitch strict +nav\n"), options);
  EXPECT_EQ(2u, result.log.size());
  ASSERT_TRUE(first_slow);
  EXPECT_GE(last_slow - *first_slow, SLOW_IO_TIME - milliseconds(20));
  trace().set_slow_io(SlowIo::Off);
}

TEST(Limits, hardware_reports_a_command_beyond_its_joint_limit_and_applies_one_clamped_into_it)
{
  trace().reset();
  faults().clear();
  ArmPlugin arm;
  const rclcpp_lifecycle::State state;
  ASSERT_EQ(CallbackReturn::SUCCESS, arm->on_configure(state));
  ASSERT_EQ(CallbackReturn::SUCCESS, arm->on_activate(state));
  ASSERT_EQ(
    hardware_interface::return_type::OK, arm->perform_command_mode_switch({"a1/velocity"}, {}));
  const auto applied = [&](double value)
  {
    EXPECT_TRUE(arm.command("a1/velocity")->set_value(value));
    trace().take_events();
    EXPECT_EQ(hardware_interface::return_type::OK, arm.write());
    std::vector<EventKind> kinds;
    for (const auto & event : trace().take_events())
    {
      kinds.push_back(event.kind);
    }
    return kinds;
  };
  EXPECT_EQ(std::vector<EventKind>{EventKind::LimitExceeded}, applied(rig::LIMIT_PROBE));
  EXPECT_EQ(std::vector<EventKind>{EventKind::LimitExceeded}, applied(-rig::LIMIT_PROBE));
  EXPECT_EQ(std::vector<EventKind>{}, applied(rig::VELOCITY_LIMIT));
  EXPECT_EQ(std::vector<EventKind>{}, applied(-rig::VELOCITY_LIMIT));
  // Upstream's limiter caps a velocity by the distance to the position limit per cycle
  EXPECT_EQ(std::vector<EventKind>{}, applied(rig::POSITION_LIMIT / 0.01));
  EXPECT_EQ(std::vector<EventKind>{EventKind::UnknownValue}, applied(0.5))
    << "only a value beyond every token carries none";
}

TEST(Torn, hardware_reports_one_write_of_a_controllers_commands_from_two_updates)
{
  trace().reset();
  faults().clear();
  ArmPlugin arm;
  const rclcpp_lifecycle::State state;
  ASSERT_EQ(CallbackReturn::SUCCESS, arm->on_configure(state));
  ASSERT_EQ(CallbackReturn::SUCCESS, arm->on_activate(state));
  ASSERT_EQ(
    hardware_interface::return_type::OK,
    arm->perform_command_mode_switch({"a1/velocity", "a2/velocity"}, {}));
  const uint16_t arm_vel = *rig::controller_index("arm_vel");
  trace().set_controller_state(arm_vel, Lifecycle::Active);
  const auto applied = [&](bool torn)
  {
    trace().begin_cycle();
    const uint64_t a1 = trace().mint(arm_vel, *rig::interface_id("a1/velocity"), 0);
    if (torn)
    {
      trace().begin_cycle();
    }
    const uint64_t a2 = trace().mint(arm_vel, *rig::interface_id("a2/velocity"), 0);
    EXPECT_TRUE(arm.command("a1/velocity")->set_value(encode_token(a1)));
    EXPECT_TRUE(arm.command("a2/velocity")->set_value(encode_token(a2)));
    trace().take_events();
    EXPECT_EQ(hardware_interface::return_type::OK, arm.write());
    std::vector<EventKind> kinds;
    for (const auto & event : trace().take_events())
    {
      kinds.push_back(event.kind);
    }
    return kinds;
  };
  EXPECT_EQ(std::vector<EventKind>{}, applied(false));
  EXPECT_EQ(std::vector<EventKind>{EventKind::TornCommand}, applied(true));
}

TEST(HardwareCalls, io_of_hardware_not_active_or_after_a_fault_is_reported)
{
  trace().reset();
  faults().clear();
  ArmPlugin arm;
  const rclcpp_lifecycle::State state;
  arm.read();
  EXPECT_TRUE(took_event(EventKind::IoWhileInactive)) << "unconfigured hardware is not read";
  ASSERT_EQ(CallbackReturn::SUCCESS, arm->on_configure(state));
  arm.read();
  EXPECT_FALSE(took_event(EventKind::IoWhileInactive)) << "inactive hardware may be read";
  arm.write();
  EXPECT_TRUE(took_event(EventKind::IoWhileInactive)) << "inactive hardware is not written";
  ASSERT_EQ(CallbackReturn::SUCCESS, arm->on_activate(state));
  faults().arm({TargetKind::Hardware, *rig::hardware_index("arm")}, Hook::Read, Fault::Error);
  EXPECT_EQ(hardware_interface::return_type::ERROR, arm.read());
  EXPECT_FALSE(took_event(EventKind::IoAfterFault));
  arm.write();
  EXPECT_TRUE(took_event(EventKind::IoAfterFault));
  ASSERT_EQ(CallbackReturn::SUCCESS, arm->on_error(state));
  ASSERT_EQ(CallbackReturn::SUCCESS, arm->on_configure(state));
  arm.read();
  EXPECT_FALSE(took_event(EventKind::IoAfterFault)) << "a transition handles the fault";
}

TEST(HardwareCalls, two_modes_left_active_on_one_joint_are_reported)
{
  trace().reset();
  faults().clear();
  ArmPlugin arm;
  const rclcpp_lifecycle::State state;
  ASSERT_EQ(CallbackReturn::SUCCESS, arm->on_configure(state));
  ASSERT_EQ(CallbackReturn::SUCCESS, arm->on_activate(state));
  arm->perform_command_mode_switch({"a1/velocity", "a2/position"}, {});
  EXPECT_FALSE(took_event(EventKind::ModeConflict));
  arm->perform_command_mode_switch({"a1/position"}, {});
  EXPECT_TRUE(took_event(EventKind::ModeConflict));
  arm->perform_command_mode_switch({"a1/position"}, {"a1/velocity"});
  EXPECT_FALSE(took_event(EventKind::ModeConflict)) << "one mode replaces the other";
}

TEST(HardwareCalls, a_call_into_hardware_busy_with_another_is_reported)
{
  trace().reset();
  faults().clear();
  ArmPlugin arm;
  const rclcpp_lifecycle::State state;
  ASSERT_EQ(CallbackReturn::SUCCESS, arm->on_configure(state));
  ASSERT_EQ(CallbackReturn::SUCCESS, arm->on_activate(state));
  // A held read of sync hardware stays inside the plugin until released
  const auto deadline = ControlLoop::Clock::now() + std::chrono::seconds(5);
  trace().hold_reads(deadline);
  std::thread reader([&] { arm.read(); });
  // Whichever call comes second sees the other inside the plugin
  bool reported = false;
  while (!reported && ControlLoop::Clock::now() < deadline)
  {
    arm.write();
    reported = took_event(EventKind::ConcurrentHardwareCall);
  }
  trace().release_reads();
  reader.join();
  EXPECT_TRUE(reported);
  arm.read();
  arm.write();
  EXPECT_FALSE(took_event(EventKind::ConcurrentHardwareCall));
}

TEST(Limits, controllers_probe_only_hardware_commands)
{
  trace().reset();
  const auto probes = [](const char * name)
  {
    ControllerTracer controller(name);
    size_t count = 0;
    for (uint64_t i = 0; i < 4 * rig::LIMIT_PROBE_PERIOD; ++i)
    {
      count += controller.mint_command(0, 0) == rig::LIMIT_PROBE;
    }
    return count;
  };
  EXPECT_GE(probes("arm_vel"), 3u) << "arm_vel commands arm";
  EXPECT_EQ(0u, probes("wrist")) << "wrist commands a reference of arm_vel";
}

TEST(Description, accepting_duplicate_interfaces_is_reported)
{
  RunOptions options;
  options.config = default_config();
  const auto description = rig::urdf_with_duplicate_interfaces();
  // SlowAsyncCm brings all hardware up whatever the description, IdleCm runs none
  const auto accepted = run_rejected_description(
    [](const CmConfig &) { return std::make_unique<SlowAsyncCm>(); }, description, options);
  ASSERT_FALSE(accepted.failures.empty());
  EXPECT_NE(std::string::npos, accepted.failures.front().find("[outcome]"))
    << accepted.failures.front();
  const auto rejected = run_rejected_description(
    [](const CmConfig &) { return std::make_unique<IdleCm>(); }, description, options);
  EXPECT_TRUE(rejected.passed()) << rejected.failures.front();
  const auto thrown = run_rejected_description(
    [](const CmConfig &) -> std::unique_ptr<CmUnderTest>
    { throw std::invalid_argument("duplicate interface a1/position"); }, description, options);
  EXPECT_TRUE(thrown.passed());
}

/// Brings up the startup profile by itself, 50 ms after it was created, and reports the truth
/// report_delay later.
class StartupCm : public SlowAsyncCm
{
public:
  explicit StartupCm(milliseconds report_delay = milliseconds(0))
  : reported_from_(ControlLoop::Clock::now() + milliseconds(50) + report_delay)
  {
    const auto & profile = rig::startup_profile();
    for (const auto & name : profile.controllers_loaded)
    {
      trace().set_controller_state(*rig::controller_index(name), Lifecycle::Inactive);
    }
    startup_ = std::thread(
      [profile]
      {
        std::this_thread::sleep_for(milliseconds(50));
        for (const auto & name : profile.controllers_active)
        {
          trace().set_controller_state(*rig::controller_index(name), Lifecycle::Active);
        }
        set_chained_modes();
      });
  }

  ~StartupCm() override { startup_.join(); }

  ReportedState reported() override
  {
    ReportedState state;
    const bool done = ControlLoop::Clock::now() >= reported_from_;
    for (uint16_t i = 0; i < rig::controllers().size(); ++i)
    {
      const auto truth = trace().controller(i).state;
      if (truth != Lifecycle::Unloaded)
      {
        state.controllers[rig::controllers()[i].name] = done ? truth : Lifecycle::Inactive;
      }
    }
    for (uint16_t i = 0; i < rig::hardware().size(); ++i)
    {
      state.hardware[rig::hardware()[i].name] = trace().hardware(i).state;
    }
    return state;
  }

protected:
  const ControlLoop::Clock::time_point reported_from_;

private:
  std::thread startup_;
};

TEST(Setup, ends_once_the_startup_profile_is_reached)
{
  RunOptions options;
  options.config = default_config();
  options.capabilities.startup = true;
  // StartupCm never applies its claims, so settling would wait out the lag for them
  options.skipped_checks = {check::LIVENESS};
  const auto start = ControlLoop::Clock::now();
  const auto result =
    run_script([](const CmConfig &) { return std::make_unique<StartupCm>(); }, {}, options);
  EXPECT_LT(ControlLoop::Clock::now() - start, options.config.switch_timeout);
  for (const auto & failure : result.failures)
  {
    EXPECT_EQ(std::string::npos, failure.find("[setup]")) << failure;
  }
}

// Bringing up many controllers on a starved machine takes longer than any switch
TEST(Setup, waits_for_the_startup_longer_than_the_switch_timeout)
{
  RunOptions options;
  options.config = default_config();
  options.config.switch_timeout = milliseconds(1);
  options.capabilities.startup = true;
  const auto result =
    run_script([](const CmConfig &) { return std::make_unique<StartupCm>(); }, {}, options);
  for (const auto & failure : result.failures)
  {
    EXPECT_EQ(std::string::npos, failure.find("[setup]")) << failure;
  }
}

TEST(Setup, waits_until_the_startup_profile_is_reported)
{
  RunOptions options;
  options.config = default_config();
  options.capabilities.startup = true;
  const auto result = run_script(
    [](const CmConfig &) { return std::make_unique<StartupCm>(milliseconds(100)); }, {}, options);
  for (const auto & failure : result.failures)
  {
    EXPECT_EQ(std::string::npos, failure.find("[status] setup")) << failure;
  }
}

/// Applies the hardware interfaces of the startup controllers 100 ms after activating them, like a
/// CM that switches the modes a supervisor step after the activation.
class LateModeSwitchCm : public StartupCm
{
public:
  LateModeSwitchCm()
  : modes_(
      []
      {
        std::this_thread::sleep_for(milliseconds(150));
        set_startup_interfaces_live(true);
      })
  {
  }

  ~LateModeSwitchCm() override { modes_.join(); }

  void cycle() override
  {
    StartupCm::cycle();
    for (uint16_t i = 0; i < rig::interface_count(); ++i)
    {
      if (trace().interface(i).live)
      {
        trace().set_interface_applied(i);
      }
    }
  }

  void shutdown() override
  {
    set_startup_interfaces_live(false);
    StartupCm::shutdown();
  }

private:
  static void set_startup_interfaces_live(bool live)
  {
    for (const auto & name : rig::startup_profile().controllers_active)
    {
      for (const auto & interface : rig::controller(name).command_interfaces)
      {
        if (rig::hardware_of_interface(interface))
        {
          trace().set_interface_live(*rig::interface_id(interface), live);
        }
      }
    }
  }

  std::thread modes_;
};

TEST(Setup, waits_for_the_startup_interfaces_applied_after_the_activation)
{
  RunOptions options;
  options.config = default_config();
  options.capabilities.startup = true;
  const auto unapplied = [&](const CmFactory & factory)
  {
    const auto result = run_script(factory, {}, options);
    return std::any_of(
      result.failures.begin(), result.failures.end(), [](const std::string & failure)
      { return failure.find("[liveness] setup") != std::string::npos; });
  };
  EXPECT_FALSE(unapplied([](const CmConfig &) { return std::make_unique<LateModeSwitchCm>(); }));
  EXPECT_TRUE(unapplied([](const CmConfig &) { return std::make_unique<StartupCm>(); }))
    << "interfaces never applied are reported";
}

TEST(Settle, does_not_wait_for_claims_whose_liveness_is_not_recorded)
{
  const auto duration = [](const RunOptions & options)
  {
    const auto start = ControlLoop::Clock::now();
    run_script([](const CmConfig &) { return std::make_unique<StartupCm>(); }, {}, options);
    return ControlLoop::Clock::now() - start;
  };
  RunOptions skipped;
  skipped.config = default_config();
  skipped.capabilities.startup = true;
  skipped.skipped_checks = {check::LIVENESS};
  RunOptions gates_only = skipped;
  gates_only.skipped_checks = {};
  gates_only.gates_only = true;
  // Waiting for the never applied claims would add the whole lag, 200 ms or more
  EXPECT_LT(duration(gates_only), duration(skipped) + milliseconds(100));
}

/// Stops everything on hardware without power, and its first recovery throws.
class FailingRecoveryCm : public StartupCm
{
public:
  void cycle() override
  {
    StartupCm::cycle();
    if (trace().power())
    {
      return;
    }
    for (uint16_t i = 0; i < rig::controllers().size(); ++i)
    {
      if (trace().controller(i).state == Lifecycle::Active)
      {
        trace().set_controller_state(i, Lifecycle::Inactive);
      }
    }
    for (uint16_t i = 0; i < rig::hardware().size(); ++i)
    {
      if (rig::hardware()[i].faults_on_power_loss)
      {
        trace().set_hardware_state(i, Lifecycle::Unconfigured);
      }
    }
  }

  bool recover_hardware(const std::string & name) override
  {
    if (!recovered_once_.exchange(true))
    {
      throw std::runtime_error("recovery failed");
    }
    trace().set_hardware_state(*rig::hardware_index(name), Lifecycle::Active);
    return true;
  }

private:
  std::atomic<bool> recovered_once_{false};
};

TEST(PowerRecovery, reports_only_the_hardware_whose_recovery_ran)
{
  RunOptions options;
  options.config = default_config();
  options.capabilities.startup = true;
  options.skipped_checks = {check::EVENT,    check::CONSISTENCY, check::LIVENESS, check::STATUS,
                            check::ACTIVITY, check::OUTCOME,     check::SHUTDOWN, check::POWER_CUT};
  const auto result = run_script(
    [](const CmConfig &) { return std::make_unique<FailingRecoveryCm>(); },
    parse_script("powercut on\npowercut off\n"), options);
  std::vector<std::string> recovery;
  std::copy_if(
    result.failures.begin(), result.failures.end(), std::back_inserter(recovery),
    [](const std::string & failure)
    { return failure.find("[power_recovery]") != std::string::npos; });
  ASSERT_EQ(1u, recovery.size()) << testing::PrintToString(result.failures);
  // arm comes first and its recovery threw, base and imu were never attempted
  EXPECT_NE(std::string::npos, recovery.front().find(": [arm] did not recover"))
    << recovery.front();
}

/// Its shutdown leaves a hang running, like a component whose on_deactivate never returns, and it
/// counts the cycles it gets after shutdown returned.
class HangingShutdownCm : public StartupCm
{
public:
  ~HangingShutdownCm() override { hang_.join(); }

  void cycle() override
  {
    if (shut_down_)
    {
      ++cycles_after_shutdown;
    }
    StartupCm::cycle();
  }

  void shutdown() override
  {
    const Target arm{TargetKind::Hardware, 0};
    trace().begin_hang(arm);
    hang_ = std::thread(
      [arm]
      {
        std::this_thread::sleep_for(milliseconds(100));
        trace().end_hang(arm);
      });
    StartupCm::shutdown();
    shut_down_ = true;
  }

  static inline std::atomic<int> cycles_after_shutdown{0};

private:
  std::atomic<bool> shut_down_{false};
  std::thread hang_;
};

TEST(Shutdown, no_cycle_runs_after_shutdown_returned)
{
  RunOptions options;
  options.config = default_config();
  options.capabilities.startup = true;
  options.skipped_checks = {check::EVENT,  check::CONSISTENCY, check::LIVENESS,
                            check::STATUS, check::ACTIVITY,    check::SHUTDOWN};
  for (const auto mode : {Mode::Stepped, Mode::Threaded})
  {
    options.mode = mode;
    HangingShutdownCm::cycles_after_shutdown = 0;
    run_script(
      [](const CmConfig &) { return std::make_unique<HangingShutdownCm>(); },
      parse_script("shutdown\n"), options);
    // Only the few cycles that start before the harness sees the return, not the 100 ms of hang
    EXPECT_LT(HangingShutdownCm::cycles_after_shutdown.load(), 10);
  }
}

/// Its shutdown stops everything but base, whose read takes 50 ms and hangs if asked. It finishes
/// base the given time after the read returned or never, like an async owner that returns to the
/// controller manager only after the injected hang counted as over.
class HungReadAtShutdownCm : public StartupCm
{
public:
  HungReadAtShutdownCm(std::optional<milliseconds> finish_after, bool hang)
  : finish_after_(finish_after), hang_(hang)
  {
  }

  ~HungReadAtShutdownCm() override
  {
    if (read_.joinable())
    {
      read_.join();
    }
  }

  void shutdown() override
  {
    for (uint16_t i = 0; i < rig::controllers().size(); ++i)
    {
      if (trace().controller(i).state == Lifecycle::Active)
      {
        trace().set_controller_state(i, Lifecycle::Inactive);
      }
    }
    const uint16_t base = *rig::hardware_index("base");
    for (uint16_t i = 0; i < rig::hardware().size(); ++i)
    {
      if (i != base)
      {
        trace().set_hardware_state(i, Lifecycle::Inactive);
      }
    }
    if (hang_)
    {
      trace().begin_hang({TargetKind::Hardware, base});
    }
    read_ = std::thread(
      [base, finish_after = finish_after_, hang = hang_]
      {
        std::this_thread::sleep_for(milliseconds(50));
        if (hang)
        {
          trace().end_hang({TargetKind::Hardware, base});
        }
        if (finish_after)
        {
          std::this_thread::sleep_for(*finish_after);
          trace().set_hardware_state(base, Lifecycle::Inactive);
        }
      });
  }

private:
  const std::optional<milliseconds> finish_after_;
  const bool hang_;
  std::thread read_;
};

TEST(Shutdown, waits_for_a_component_finished_after_its_hang_returned_but_not_forever)
{
  RunOptions options;
  options.config = default_config();
  options.capabilities.startup = true;
  options.skipped_checks = {check::LIVENESS, check::ACTIVITY};
  for (const auto mode : {Mode::Stepped, Mode::Threaded})
  {
    options.mode = mode;
    SCOPED_TRACE(mode == Mode::Threaded ? "threaded" : "stepped");
    for (const auto * script : {"", "shutdown\n"})
    {
      const auto run = [&](std::optional<milliseconds> finish_after, bool hang)
      {
        return run_script(
          [finish_after, hang](const CmConfig &)
          { return std::make_unique<HungReadAtShutdownCm>(finish_after, hang); },
          parse_script(script), options);
      };
      const auto late = run(milliseconds(20), true);
      EXPECT_TRUE(late.passed()) << script << testing::PrintToString(late.failures);
      const auto never = run(std::nullopt, true);
      EXPECT_TRUE(failed_with(never, "hardware [base] is still active after shutdown"))
        << script << testing::PrintToString(never.failures);
      // Without a hang the controller manager owes a finished component when shutdown returned
      const auto unhung = run(milliseconds(20), false);
      EXPECT_TRUE(failed_with(unhung, "hardware [base] is still active after shutdown"))
        << script << testing::PrintToString(unhung.failures);
    }
  }
}

/// Once settling began, a callback of jsb hangs for 100 ms and leaves jsb inactive meanwhile, like
/// a fault stop that hits an injected hang while the harness settles.
class HangWhileSettlingCm : public StartupCm
{
public:
  ~HangWhileSettlingCm() override
  {
    if (hang_.joinable())
    {
      hang_.join();
    }
  }

  void cycle() override
  {
    StartupCm::cycle();
    if (hang_.joinable() || ControlLoop::Clock::now() < reported_from_ + milliseconds(2))
    {
      return;
    }
    const uint16_t jsb = *rig::controller_index("jsb");
    trace().begin_hang({TargetKind::Controller, jsb});
    trace().set_controller_state(jsb, Lifecycle::Inactive);
    hang_ = std::thread(
      [jsb]
      {
        std::this_thread::sleep_for(milliseconds(100));
        trace().set_controller_state(jsb, Lifecycle::Active);
        trace().end_hang({TargetKind::Controller, jsb});
      });
  }

private:
  std::thread hang_;
};

TEST(Settle, waits_out_a_hang_that_starts_while_settling)
{
  RunOptions options;
  options.config = default_config();
  options.capabilities.startup = true;
  const auto result = run_script(
    [](const CmConfig &) { return std::make_unique<HangWhileSettlingCm>(); }, {}, options);
  for (const auto & failure : result.failures)
  {
    EXPECT_EQ(std::string::npos, failure.find("[setup]")) << failure;
  }
}

/// Once settling began, a mode switch call of base runs for 100 ms and starts its interfaces only
/// at its end, like a slow async perform that the controller manager runs after the switch
/// returned.
class SlowPerformCm : public StartupCm
{
public:
  ~SlowPerformCm() override
  {
    if (perform_.joinable())
    {
      perform_.join();
    }
  }

  void cycle() override
  {
    StartupCm::cycle();
    if (perform_.joinable() || ControlLoop::Clock::now() < reported_from_ + milliseconds(2))
    {
      return;
    }
    const uint16_t jsb = *rig::controller_index("jsb");
    trace().begin_call();
    trace().set_controller_state(jsb, Lifecycle::Inactive);
    perform_ = std::thread(
      [jsb]
      {
        std::this_thread::sleep_for(milliseconds(100));
        trace().set_controller_state(jsb, Lifecycle::Active);
        trace().end_call();
      });
  }

private:
  std::thread perform_;
};

TEST(Settle, waits_for_a_running_lifecycle_or_mode_switch_call)
{
  RunOptions options;
  options.config = default_config();
  options.capabilities.startup = true;
  const auto result =
    run_script([](const CmConfig &) { return std::make_unique<SlowPerformCm>(); }, {}, options);
  for (const auto & failure : result.failures)
  {
    EXPECT_EQ(std::string::npos, failure.find("[setup]")) << failure;
  }
}

/// Its shutdown waits for a control cycle, like a controller manager that hands shutdown to RT.
class ShutdownOnRtCm : public StartupCm
{
public:
  void cycle() override
  {
    StartupCm::cycle();
    cycled_ = true;
  }

  void shutdown() override
  {
    // The stall begins one cycle into the operation
    std::this_thread::sleep_for(milliseconds(20));
    cycled_ = false;
    // Outlasts a stall, which ends at the time limit of the shutdown
    const auto give_up = ControlLoop::Clock::now() + CmConfig{}.switch_timeout + 2 * TIMEOUT_MARGIN;
    while (!cycled_ && ControlLoop::Clock::now() < give_up)
    {
      std::this_thread::sleep_for(milliseconds(1));
    }
    StartupCm::shutdown();
  }

private:
  std::atomic<bool> cycled_{false};
};

TEST(Stall, holds_the_loop_during_a_stalled_shutdown)
{
  RunOptions options;
  options.config = default_config();
  options.capabilities.startup = true;
  options.skipped_checks = {check::EVENT,    check::CONSISTENCY, check::LIVENESS, check::STATUS,
                            check::ACTIVITY, check::SHUTDOWN,    check::SETUP,    check::OUTCOME};
  const auto timeouts = [&](const char * script)
  {
    const auto result = run_script(
      [](const CmConfig &) { return std::make_unique<ShutdownOnRtCm>(); }, parse_script(script),
      options);
    return std::count_if(
      result.failures.begin(), result.failures.end(),
      [](const std::string & f) { return f.find("[timeout]") != std::string::npos; });
  };
  EXPECT_EQ(0, timeouts("shutdown\n"));
  EXPECT_EQ(1, timeouts("shutdown stall\n"));
}

/// Its cycle reads like sync hardware does and records whether the harness held it there.
class HeldReadCm : public StartupCm
{
public:
  void cycle() override
  {
    StartupCm::cycle();
    if (trace().wait_while_held())
    {
      held = true;
    }
  }

  static inline std::atomic<bool> held{false};
};

TEST(Stall, holds_the_loop_inside_a_read_until_the_switch_returns)
{
  RunOptions options;
  options.config = default_config();
  options.capabilities.startup = true;
  options.skipped_checks = {check::EVENT,    check::CONSISTENCY, check::LIVENESS, check::STATUS,
                            check::ACTIVITY, check::SHUTDOWN,    check::SETUP,    check::OUTCOME};
  for (const auto mode : {Mode::Stepped, Mode::Threaded})
  {
    options.mode = mode;
    HeldReadCm::held = false;
    const auto start = ControlLoop::Clock::now();
    const auto result = run_script(
      [](const CmConfig &) { return std::make_unique<HeldReadCm>(); },
      parse_script("switch strict +monitor hold\n"), options);
    EXPECT_TRUE(HeldReadCm::held.load());
    EXPECT_TRUE(result.passed()) << result.failures.front();
    // The hold ends when the switch returns, not at its limit
    EXPECT_LT(ControlLoop::Clock::now() - start, milliseconds(1500));
  }
}

/// Its switch waits for a completed cycle, like a controller manager whose cycle holds a lock
/// across read that the switch needs.
class SwitchOnRtCm : public HeldReadCm
{
public:
  void cycle() override
  {
    HeldReadCm::cycle();
    cycled_ = true;
  }

  bool switch_controllers(
    const std::vector<std::string> &, const std::vector<std::string> &, Strictness) override
  {
    // The hold begins one cycle into the operation
    std::this_thread::sleep_for(milliseconds(20));
    cycled_ = false;
    // Outlasts a hold, which ends at the time limit of the switch
    const auto give_up = ControlLoop::Clock::now() + CmConfig{}.switch_timeout + 2 * TIMEOUT_MARGIN;
    while (!cycled_ && ControlLoop::Clock::now() < give_up)
    {
      std::this_thread::sleep_for(milliseconds(1));
    }
    return true;
  }

private:
  std::atomic<bool> cycled_{false};
};

TEST(Stall, a_switch_that_waits_for_the_held_read_times_out)
{
  RunOptions options;
  options.config = default_config();
  options.capabilities.startup = true;
  options.skipped_checks = {check::EVENT,    check::CONSISTENCY, check::LIVENESS, check::STATUS,
                            check::ACTIVITY, check::SHUTDOWN,    check::SETUP,    check::OUTCOME};
  for (const auto mode : {Mode::Stepped, Mode::Threaded})
  {
    options.mode = mode;
    const auto timeouts = [&](const char * script)
    {
      const auto result = run_script(
        [](const CmConfig &) { return std::make_unique<SwitchOnRtCm>(); }, parse_script(script),
        options);
      return std::count_if(
        result.failures.begin(), result.failures.end(),
        [](const std::string & f) { return f.find("[timeout]") != std::string::npos; });
    };
    EXPECT_EQ(0, timeouts("switch strict +monitor\n"));
    EXPECT_EQ(1, timeouts("switch strict +monitor hold\n"));
  }
}

TEST(Stall, ends_when_the_switch_returns)
{
  RunOptions options;
  options.config = default_config();
  options.capabilities.startup = true;
  // Only the duration matters here, the idle controller manager fails most checks
  options.skipped_checks = {
    check::EVENT,    check::CONSISTENCY, check::STATUS,        check::ACTIVITY,    check::OUTCOME,
    check::LIVENESS, check::TIMEOUT,     check::RT_ALLOCATION, check::RT_BLOCKING, check::SETUP,
    check::SHUTDOWN, check::POWER_CUT,   check::POWER_RECOVERY};
  const Script script = parse_script("switch strict +monitor stall\n");
  const auto start = ControlLoop::Clock::now();
  const auto result =
    run_script([](const CmConfig &) { return std::make_unique<StartupCm>(); }, script, options);
  EXPECT_TRUE(result.passed());
  // The run waits for the activity topic twice, 500 ms each, the full stall would add 1.5 s more
  EXPECT_LT(ControlLoop::Clock::now() - start, milliseconds(1500));
}

bool has_failure(const RunResult & result, const std::string & text)
{
  return std::any_of(
    result.failures.begin(), result.failures.end(),
    [&](const std::string & failure) { return failure.find(text) != std::string::npos; });
}

/// Rejects an unload whose on_shutdown hangs and removes the controller once it returns. A
/// configure cleans up first and its activity topic shows the cleanup for a while.
class LateLifecycleCm : public StartupCm
{
public:
  ~LateLifecycleCm() override
  {
    if (shutdown_.joinable())
    {
      shutdown_.join();
    }
  }

  bool configure(const std::string & name) override
  {
    const uint16_t index = *rig::controller_index(name);
    trace().set_controller_state(index, Lifecycle::Unconfigured);
    trace().set_controller_state(index, Lifecycle::Inactive);
    std::lock_guard<std::mutex> lock(mutex_);
    lagging_ = name;
    lag_until_ = ControlLoop::Clock::now() + milliseconds(100);
    return true;
  }

  bool unload(const std::string & name) override
  {
    const Target target{TargetKind::Controller, *rig::controller_index(name)};
    trace().begin_hang(target);
    shutdown_ = std::thread(
      [target]
      {
        std::this_thread::sleep_for(milliseconds(300));
        trace().set_controller_state(target.index, Lifecycle::Unloaded);
        trace().end_hang(target);
      });
    return false;
  }

  ReportedState activity() override
  {
    ReportedState state = reported();
    std::lock_guard<std::mutex> lock(mutex_);
    if (ControlLoop::Clock::now() < lag_until_)
    {
      state.controllers[lagging_] = Lifecycle::Unconfigured;
    }
    return state;
  }

  // StartupCm::shutdown leaves the startup profile active, which the shutdown check reports
  void shutdown() override
  {
    for (uint16_t i = 0; i < rig::controllers().size(); ++i)
    {
      if (trace().controller(i).state == Lifecycle::Active)
      {
        trace().set_controller_state(i, Lifecycle::Inactive);
      }
    }
    StartupCm::shutdown();
  }

private:
  std::thread shutdown_;
  std::mutex mutex_;
  std::string lagging_;
  ControlLoop::Clock::time_point lag_until_;
};

TEST(Unchecked, a_lifecycle_step_may_finish_during_the_next_step)
{
  RunOptions options;
  options.config = default_config();
  options.capabilities.startup = true;
  options.skipped_checks = {check::LIVENESS};
  const auto result = run_script(
    [](const CmConfig &) { return std::make_unique<LateLifecycleCm>(); },
    parse_script("unload monitor nowait\nfault controller:nav:update:throw\n"), options);
  EXPECT_TRUE(result.passed()) << testing::PrintToString(result.failures);
}

TEST(Unchecked, the_next_step_may_report_the_states_a_lifecycle_step_passed_through)
{
  RunOptions options;
  options.config = default_config();
  options.capabilities.startup = true;
  options.skipped_checks = {check::LIVENESS};
  const auto result = run_script(
    [](const CmConfig &) { return std::make_unique<LateLifecycleCm>(); },
    parse_script("configure nav nowait\nfault controller:teleop:update:throw\n"), options);
  EXPECT_TRUE(result.passed()) << testing::PrintToString(result.failures);
}

/// Its shutdown throws after bringing everything down.
class ThrowingShutdownCm : public StartupCm
{
public:
  void shutdown() override
  {
    StartupCm::shutdown();
    throw std::runtime_error("shutdown failed");
  }
};

/// Its shutdown returns only after the given time.
class SlowShutdownCm : public StartupCm
{
public:
  explicit SlowShutdownCm(milliseconds duration) : duration_(duration) {}

  void shutdown() override
  {
    std::this_thread::sleep_for(duration_);
    StartupCm::shutdown();
  }

private:
  const milliseconds duration_;
};

TEST(Shutdown, the_implicit_shutdown_reports_a_throw_and_a_hang)
{
  RunOptions options;
  options.config = default_config();
  // Shortens the hang limit of an operation to 5.6 s
  options.config.switch_timeout = milliseconds(100);
  options.capabilities.startup = true;
  options.skipped_checks = {check::LIVENESS};
  const auto thrown = run_script(
    [](const CmConfig &) { return std::make_unique<ThrowingShutdownCm>(); }, {}, options);
  EXPECT_TRUE(has_failure(thrown, "[exception] shutdown: the operation threw: shutdown failed"))
    << testing::PrintToString(thrown.failures);
  const auto hung = run_script(
    [](const CmConfig &) { return std::make_unique<SlowShutdownCm>(milliseconds(6000)); }, {},
    options);
  EXPECT_TRUE(has_failure(hung, "[hang] shutdown")) << testing::PrintToString(hung.failures);
}

/// Toggles monitor between inactive and active in every cycle.
class FlappingCm : public StartupCm
{
public:
  void cycle() override
  {
    StartupCm::cycle();
    const uint16_t monitor = *rig::controller_index("monitor");
    const bool active = trace().controller(monitor).state == Lifecycle::Active;
    trace().set_controller_state(monitor, active ? Lifecycle::Inactive : Lifecycle::Active);
  }
};

TEST(Settle, reports_what_kept_it_from_settling_at_its_deadline)
{
  RunOptions options;
  options.config = default_config();
  options.capabilities.startup = true;
  const auto result =
    run_script([](const CmConfig &) { return std::make_unique<FlappingCm>(); }, {}, options);
  EXPECT_TRUE(
    has_failure(result, "[settle] setup: did not settle in time, the state kept changing"))
    << testing::PrintToString(result.failures);
  EXPECT_EQ(1u, result.stats.settle_deadline_hits);
}

/// Its reports throw once a load began.
class ThrowingReportCm : public StartupCm
{
public:
  bool load(const std::string &) override
  {
    loading_ = true;
    // Long enough for the sampler to see it
    std::this_thread::sleep_for(milliseconds(20));
    return true;
  }

  ReportedState reported() override
  {
    if (loading_)
    {
      throw std::runtime_error("no report");
    }
    return StartupCm::reported();
  }

private:
  std::atomic<bool> loading_{false};
};

TEST(Status, a_report_that_throws_during_an_unchecked_step_is_reported)
{
  RunOptions options;
  options.config = default_config();
  options.capabilities.startup = true;
  options.skipped_checks = {check::LIVENESS, check::ACTIVITY};
  const auto result = run_script(
    [](const CmConfig &) { return std::make_unique<ThrowingReportCm>(); },
    parse_script("load nav nowait\n"), options);
  EXPECT_TRUE(
    has_failure(result, "[exception] step 1 'load nav nowait': reporting the state threw"))
    << testing::PrintToString(result.failures);
}

/// Reports monitor inactive from the given call on.
class CountingReportCm : public IdleCm
{
public:
  explicit CountingReportCm(int change_at) : change_at_(change_at) {}

  ReportedState reported() override
  {
    ReportedState state;
    if (++calls_ >= change_at_)
    {
      state.controllers["monitor"] = Lifecycle::Inactive;
    }
    return state;
  }

private:
  const int change_at_;
  std::atomic<int> calls_{0};
};

TEST(Status, the_sampler_keeps_only_changes)
{
  CountingReportCm cm(20);
  StatusSampler sampler(cm);
  std::this_thread::sleep_for(milliseconds(100));
  sampler.stop();
  ASSERT_EQ(2u, sampler.samples().size());
  EXPECT_TRUE(sampler.samples()[0].reported.controllers.empty());
  EXPECT_EQ(1u, sampler.samples()[1].reported.controllers.count("monitor"));
}

/// Takes the faults of a flipper mode switch and stops flipper_pos, but never starts flipper_vel.
class SwitchIgnoringCm : public StartupCm
{
public:
  bool switch_controllers(
    const std::vector<std::string> &, const std::vector<std::string> & deactivate,
    Strictness) override
  {
    const Target flipper{TargetKind::Hardware, *rig::hardware_index("flipper")};
    faults().take(flipper, Hook::Prepare);
    faults().take(flipper, Hook::Perform);
    for (const auto & name : deactivate)
    {
      trace().set_controller_state(*rig::controller_index(name), Lifecycle::Inactive);
    }
    set_chained_modes();
    return true;
  }
};

TEST(Outcome, a_delayed_mode_switch_excuses_no_missing_activation)
{
  RunOptions options;
  options.config = default_config();
  options.capabilities.startup = true;
  options.skipped_checks = {check::LIVENESS, check::ACTIVITY};
  const auto result = run_script(
    [](const CmConfig &) { return std::make_unique<SwitchIgnoringCm>(); },
    parse_script(
      "switch strict -flipper_pos +flipper_vel fault=hardware:flipper:perform:delay@0 "
      "expect=accept\n"),
    options);
  EXPECT_TRUE(has_failure(result, "accepted, but requested [flipper_vel] are not active"))
    << testing::PrintToString(result.failures);
}

/// Stops everything and the hardware without power the given time after it saw the cut. Its reads
/// wait like sync hardware and it answers a switch after answer_after.
class PowerCutReactionCm : public StartupCm
{
public:
  explicit PowerCutReactionCm(milliseconds reaction, milliseconds answer_after = milliseconds(20))
  : reaction_(reaction), answer_after_(answer_after)
  {
  }

  void cycle() override
  {
    StartupCm::cycle();
    trace().wait_while_held();
    if (trace().power())
    {
      return;
    }
    const auto now = ControlLoop::Clock::now();
    if (!cut_seen_)
    {
      cut_seen_ = now;
    }
    if (now - *cut_seen_ < reaction_)
    {
      return;
    }
    for (uint16_t i = 0; i < rig::controllers().size(); ++i)
    {
      if (trace().controller(i).state == Lifecycle::Active)
      {
        trace().set_controller_state(i, Lifecycle::Inactive);
      }
    }
    for (uint16_t i = 0; i < rig::hardware().size(); ++i)
    {
      if (rig::hardware()[i].faults_on_power_loss)
      {
        trace().set_hardware_state(i, Lifecycle::Unconfigured);
      }
    }
  }

  bool switch_controllers(
    const std::vector<std::string> &, const std::vector<std::string> &, Strictness) override
  {
    std::this_thread::sleep_for(answer_after_);
    return true;
  }

private:
  const milliseconds reaction_;
  const milliseconds answer_after_;
  std::optional<ControlLoop::Clock::time_point> cut_seen_;
};

TEST(PowerCut, also_a_cut_during_a_switch_must_stop_the_unpowered_controllers_within_400_ms)
{
  RunOptions options;
  options.config = default_config();
  options.capabilities.startup = true;
  options.skipped_checks = {check::EVENT,    check::CONSISTENCY, check::LIVENESS,
                            check::STATUS,   check::ACTIVITY,    check::OUTCOME,
                            check::SHUTDOWN, check::POWER_CUT,   check::SETTLE};
  const auto too_slow = [&](const char * script, milliseconds reaction)
  {
    // A cut during a held switch that answers late would find the loop parked past the bound
    const auto result = run_script(
      [reaction](const CmConfig &)
      { return std::make_unique<PowerCutReactionCm>(reaction, POWER_CUT_REACTION); },
      parse_script(script), options);
    EXPECT_GE(result.stats.unpowered_steps, 1u) << script;
    return has_failure(result, "[timeout] step 1");
  };
  for (const auto mode : {Mode::Stepped, Mode::Threaded})
  {
    options.mode = mode;
    SCOPED_TRACE(mode == Mode::Threaded ? "threaded" : "stepped");
    for (const auto * script :
         {"switch strict +monitor powercut@1\n", "switch strict +monitor stall powercut@1\n",
          "switch strict +monitor hold powercut@1\n", "switch strict +monitor hold powercut@2\n",
          "powercut on\n"})
    {
      EXPECT_FALSE(too_slow(script, milliseconds(50))) << script;
      EXPECT_TRUE(too_slow(script, 2 * POWER_CUT_REACTION)) << script;
    }
  }
}

/// Updates the controllers without power only in every nth cycle, shifted by phase and never for
/// n = 0, and reports them inactive only after twice the reaction bound, like a controller manager
/// whose on_deactivate calls wait behind a late callback of another controller.
class HeldAtCutCm : public PowerCutReactionCm
{
public:
  HeldAtCutCm(uint64_t every, uint64_t phase)
  : PowerCutReactionCm(2 * POWER_CUT_REACTION), every_(every), phase_(phase)
  {
  }

protected:
  bool updates(uint16_t controller) const override
  {
    const auto & used = rig::hardware_of(rig::controllers()[controller].name);
    const bool unpowered =
      !trace().power() &&
      std::any_of(
        used.begin(), used.end(), [](const std::string & name)
        { return rig::hardware()[*rig::hardware_index(name)].faults_on_power_loss; });
    return !unpowered || (every_ > 0 && (trace().cycle() + phase_) % every_ == 0);
  }

private:
  const uint64_t every_;
  const uint64_t phase_;
};

TEST(PowerCut, a_controller_held_at_the_cut_may_report_its_stop_later)
{
  RunOptions options;
  options.config = default_config();
  options.capabilities.startup = true;
  options.skipped_checks = {check::EVENT,    check::CONSISTENCY, check::LIVENESS,
                            check::STATUS,   check::ACTIVITY,    check::OUTCOME,
                            check::SHUTDOWN, check::POWER_CUT,   check::SETTLE};
  for (const auto mode : {Mode::Stepped, Mode::Threaded})
  {
    options.mode = mode;
    const auto too_slow = [&](uint64_t every, uint64_t phase = 0)
    {
      const auto result = run_script(
        [every, phase](const CmConfig &) { return std::make_unique<HeldAtCutCm>(every, phase); },
        parse_script("powercut on\n"), options);
      return has_failure(result, "[timeout] step 1");
    };
    EXPECT_FALSE(too_slow(0));
    EXPECT_TRUE(too_slow(1)) << "it kept updating them";
    // The run's timing fixes where the bound falls between two updates, so try three phases
    for (const uint64_t phase : {0, 7, 14})
    {
      EXPECT_TRUE(too_slow(20, phase)) << "it kept updating them every 20th cycle";
    }
  }
}

/// Its control loop blocks at the cut for longer than the power cut watcher waits, then stops
/// everything without power.
class StoppedLoopAtCutCm : public PowerCutReactionCm
{
public:
  StoppedLoopAtCutCm() : PowerCutReactionCm(milliseconds(0)) {}

  void cycle() override
  {
    if (!stopped_)
    {
      // The base reads the power again and would stop everything if the cut landed in between
      if (trace().power())
      {
        StartupCm::cycle();
        return;
      }
      stopped_ = true;
      std::this_thread::sleep_for(POWER_CUT_REACTION + HANG + std::chrono::seconds(2));
    }
    PowerCutReactionCm::cycle();
  }

private:
  bool stopped_ = false;
};

TEST(PowerCut, a_control_loop_that_stops_at_the_cut_is_reported)
{
  RunOptions options;
  options.mode = Mode::Threaded;
  options.config = default_config();
  options.capabilities.startup = true;
  options.skipped_checks = {check::EVENT,    check::CONSISTENCY, check::LIVENESS, check::STATUS,
                            check::ACTIVITY, check::OUTCOME,     check::SHUTDOWN, check::POWER_CUT,
                            check::SETTLE,   check::RT_BLOCKING};
  const auto result = run_script(
    [](const CmConfig &) { return std::make_unique<StoppedLoopAtCutCm>(); },
    parse_script("powercut on\n"), options);
  EXPECT_TRUE(
    has_failure(result, "[timeout] step 1 'powercut on': the control loop stopped cycling"))
    << testing::PrintToString(result.failures);
}

/// Recovers every hardware and records the strictness of the restart.
class RecordingRestartCm : public PowerCutReactionCm
{
public:
  explicit RecordingRestartCm(std::optional<Strictness> & restart)
  : PowerCutReactionCm(milliseconds(0)), restart_(restart)
  {
  }

  bool recover_hardware(const std::string & name) override
  {
    trace().set_hardware_state(*rig::hardware_index(name), Lifecycle::Active);
    return true;
  }

  bool switch_controllers(
    const std::vector<std::string> & activate, const std::vector<std::string> &,
    Strictness strictness) override
  {
    restart_ = strictness;
    for (const auto & name : activate)
    {
      trace().set_controller_state(*rig::controller_index(name), Lifecycle::Active);
    }
    set_chained_modes();
    return true;
  }

private:
  std::optional<Strictness> & restart_;
};

TEST(PowerRecovery, restarts_with_auto_only_if_the_controller_manager_supports_it)
{
  const auto restart_of = [](bool auto_switch)
  {
    RunOptions options;
    options.config = default_config();
    options.capabilities.startup = true;
    options.capabilities.recover_hardware = true;
    options.capabilities.auto_switch = auto_switch;
    options.skipped_checks = {check::EVENT,    check::CONSISTENCY, check::LIVENESS, check::STATUS,
                              check::ACTIVITY, check::OUTCOME,     check::SHUTDOWN};
    std::optional<Strictness> restart;
    const auto result = run_script(
      [&restart](const CmConfig &) { return std::make_unique<RecordingRestartCm>(restart); },
      parse_script("powercut on\npowercut off\n"), options);
    EXPECT_TRUE(result.passed()) << testing::PrintToString(result.failures);
    return restart;
  };
  EXPECT_EQ(Strictness::Auto, restart_of(true));
  EXPECT_EQ(Strictness::Strict, restart_of(false));
}

/// Stops everything without power at the cut, except diff_drive, whose on_deactivate hangs for
/// longer than the reaction bound. diff_drive stops the given time after the hang.
class HungStopCm : public StartupCm
{
public:
  explicit HungStopCm(milliseconds stop_after_hang) : stop_after_hang_(stop_after_hang) {}

  ~HungStopCm() override
  {
    if (stop_.joinable())
    {
      stop_.join();
    }
  }

  void cycle() override
  {
    StartupCm::cycle();
    if (trace().power() || stop_.joinable())
    {
      return;
    }
    const uint16_t diff_drive = *rig::controller_index("diff_drive");
    trace().begin_hang({TargetKind::Controller, diff_drive});
    for (uint16_t i = 0; i < rig::controllers().size(); ++i)
    {
      if (i != diff_drive && trace().controller(i).state == Lifecycle::Active)
      {
        trace().set_controller_state(i, Lifecycle::Inactive);
      }
    }
    for (uint16_t i = 0; i < rig::hardware().size(); ++i)
    {
      if (rig::hardware()[i].faults_on_power_loss)
      {
        trace().set_hardware_state(i, Lifecycle::Unconfigured);
      }
    }
    stop_ = std::thread(
      [diff_drive, stop_after_hang = stop_after_hang_]
      {
        std::this_thread::sleep_for(POWER_CUT_REACTION * 3 / 2);
        trace().end_hang({TargetKind::Controller, diff_drive});
        std::this_thread::sleep_for(stop_after_hang);
        trace().set_controller_state(diff_drive, Lifecycle::Inactive);
      });
  }

private:
  const milliseconds stop_after_hang_;
  std::thread stop_;
};

TEST(PowerCut, the_reaction_of_a_controller_hanging_in_a_callback_is_bounded_from_its_return)
{
  RunOptions options;
  options.config = default_config();
  options.capabilities.startup = true;
  options.skipped_checks = {check::EVENT,    check::CONSISTENCY, check::LIVENESS,
                            check::STATUS,   check::ACTIVITY,    check::OUTCOME,
                            check::SHUTDOWN, check::POWER_CUT,   check::SETTLE};
  const auto too_slow = [&](milliseconds stop_after_hang)
  {
    const auto result = run_script(
      [stop_after_hang](const CmConfig &) { return std::make_unique<HungStopCm>(stop_after_hang); },
      parse_script("powercut on\n"), options);
    return has_failure(result, "[timeout] step 1");
  };
  EXPECT_FALSE(too_slow(milliseconds(0))) << "no controller manager can cut the hang short";
  EXPECT_TRUE(too_slow(2 * POWER_CUT_REACTION)) << "it kept running after its callback returned";
}

/// Like HungStopCm, and arm_pos starts from a late on_activate at the cut, returning before
/// diff_drive does. It stops arm_pos the given time after its return.
class LateActivationAtCutCm : public HungStopCm
{
public:
  explicit LateActivationAtCutCm(milliseconds stop_after_return)
  : HungStopCm(milliseconds(0)), stop_after_return_(stop_after_return)
  {
  }

  ~LateActivationAtCutCm() override
  {
    if (late_.joinable())
    {
      late_.join();
    }
  }

  void cycle() override
  {
    HungStopCm::cycle();
    if (trace().power() || late_.joinable())
    {
      return;
    }
    const uint16_t arm_pos = *rig::controller_index("arm_pos");
    trace().begin_hang({TargetKind::Controller, arm_pos});
    late_ = std::thread(
      [arm_pos, stop_after_return = stop_after_return_]
      {
        std::this_thread::sleep_for(POWER_CUT_REACTION * 5 / 4);
        trace().set_controller_state(arm_pos, Lifecycle::Active);
        trace().end_hang({TargetKind::Controller, arm_pos});
        std::this_thread::sleep_for(stop_after_return);
        trace().set_controller_state(arm_pos, Lifecycle::Inactive);
      });
  }

private:
  const milliseconds stop_after_return_;
  std::thread late_;
};

TEST(PowerCut, the_reaction_of_a_controller_started_by_a_late_activation_is_bounded_from_its_return)
{
  RunOptions options;
  options.config = default_config();
  options.capabilities.startup = true;
  options.skipped_checks = {check::EVENT,    check::CONSISTENCY, check::LIVENESS,
                            check::STATUS,   check::ACTIVITY,    check::OUTCOME,
                            check::SHUTDOWN, check::POWER_CUT,   check::SETTLE};
  const auto too_slow = [&](milliseconds stop_after_return)
  {
    const auto result = run_script(
      [stop_after_return](const CmConfig &)
      { return std::make_unique<LateActivationAtCutCm>(stop_after_return); },
      parse_script("powercut on\n"), options);
    return has_failure(result, "[timeout] step 1");
  };
  EXPECT_FALSE(too_slow(milliseconds(20))) << "no controller manager can cut the activation short";
  EXPECT_TRUE(too_slow(2 * POWER_CUT_REACTION)) << "it kept running after its callback returned";
}

/// Rejects a switch after the given time and leaves wrist and its unit stopped, like a rollback
/// that the deadline cut short.
class CutRollbackCm : public StartupCm
{
public:
  explicit CutRollbackCm(milliseconds answer_after) : answer_after_(answer_after) {}

  bool switch_controllers(
    const std::vector<std::string> &, const std::vector<std::string> &, Strictness) override
  {
    std::this_thread::sleep_for(answer_after_);
    for (const auto * name : {"wrist", "arm_ik", "arm_vel"})
    {
      trace().set_controller_state(*rig::controller_index(name), Lifecycle::Inactive);
    }
    set_chained_modes();
    return false;
  }

private:
  const milliseconds answer_after_;
};

TEST(Outcome, a_switch_rejected_at_its_deadline_may_leave_the_unit_of_the_request_stopped)
{
  RunOptions options;
  options.config = default_config();
  options.config.switch_timeout = milliseconds(100);
  options.capabilities.startup = true;
  options.skipped_checks = {check::LIVENESS, check::ACTIVITY, check::STATUS};
  const auto beyond_the_request = [&](milliseconds answer_after)
  {
    const auto result = run_script(
      [answer_after](const CmConfig &) { return std::make_unique<CutRollbackCm>(answer_after); },
      parse_script("switch strict -wrist\n"), options);
    return has_failure(result, "beyond the request");
  };
  EXPECT_FALSE(beyond_the_request(options.config.switch_timeout));
  EXPECT_TRUE(beyond_the_request(milliseconds(0))) << "a rejection in time restores the unit";
}

/// Rejects every switch.
class FailClosedCm : public StartupCm
{
public:
  bool switch_controllers(
    const std::vector<std::string> &, const std::vector<std::string> &, Strictness) override
  {
    return false;
  }
};

TEST(Outcome, the_acceptance_scripts_fail_a_controller_manager_that_rejects_every_switch)
{
  RunOptions options;
  options.config = default_config();
  options.capabilities.startup = true;
  options.skipped_checks = {check::LIVENESS, check::ACTIVITY};
  std::ifstream file(std::string(SCRIPTS_DIR) + "/valid_requests_strict.txt");
  std::stringstream text;
  text << file.rdbuf();
  const Script script = parse_script(text.str());
  // So a controller manager without AUTO and FORCE_AUTO does not skip it
  for (const auto & step : script)
  {
    EXPECT_EQ(Strictness::Strict, std::get<SwitchStep>(step).strictness) << to_text(step);
  }
  const auto result =
    run_script([](const CmConfig &) { return std::make_unique<FailClosedCm>(); }, script, options);
  EXPECT_TRUE(has_failure(result, "[outcome] step 1 'switch strict +nav expect=accept': rejected"))
    << testing::PrintToString(result.failures);
  EXPECT_EQ(1u, result.stats.answers.at("switch strict").rejected);
}

TEST(Tiers, gates_only_hides_provisional_findings)
{
  const CmFactory broken = [](const CmConfig &) -> std::unique_ptr<CmUnderTest>
  { throw std::runtime_error("no controller manager"); };
  RunOptions options;
  options.config = default_config();
  EXPECT_FALSE(run_script(broken, {}, options).passed());
  options.gates_only = true;
  EXPECT_TRUE(run_script(broken, {}, options).passed());
}

}  // namespace cm_test_suite
