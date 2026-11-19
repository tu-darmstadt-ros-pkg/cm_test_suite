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
#include <cmath>
#include <limits>
#include <thread>
#include <vector>

#include "cm_test_suite/cm_under_test.hpp"
#include "cm_test_suite/faults.hpp"
#include "cm_test_suite/rig.hpp"
#include "cm_test_suite/trace.hpp"

namespace cm_test_suite
{
namespace
{
uint16_t controller(const char * name) { return *rig::controller_index(name); }

uint16_t interface(const char * name) { return *rig::interface_id(name); }

const Target ARM{TargetKind::Hardware, 0};

class TraceTest : public testing::Test
{
protected:
  void SetUp() override
  {
    trace_.reset();
    for (const auto * name : {"wrist", "arm_vel"})
    {
      trace_.set_controller_state(controller(name), Lifecycle::Active);
    }
  }

  /// The tokens wrist and arm_vel write for a3 within one cycle.
  uint64_t chain()
  {
    const uint64_t reference = trace_.mint(controller("wrist"), interface("arm_vel/a3"), 0);
    return trace_.mint(controller("arm_vel"), interface("a3/velocity"), reference);
  }

  /// Pushes every earlier token out of the ring, older than MAX_COMMAND_AGE_CYCLES.
  void overwrite_the_ring()
  {
    for (size_t i = 0; i < Trace::TOKEN_CAPACITY; ++i)
    {
      trace_.mint(controller("arm_vel"), interface("a1/velocity"), 0);
    }
    for (uint64_t i = 0; i <= MAX_COMMAND_AGE_CYCLES; ++i)
    {
      trace_.begin_cycle();
    }
  }

  std::optional<EventKind> audit(uint64_t token)
  {
    const auto event = trace_.audit(token, interface("a3/velocity"), ARM);
    return event ? std::optional<EventKind>(event->kind) : std::nullopt;
  }

  Trace trace_;
};
}  // namespace

TEST(Token, encoding_round_trips_and_rejects_non_tokens)
{
  EXPECT_EQ(42u, decode_token(encode_token(42)));
  EXPECT_EQ(0u, decode_token(0.5));
  EXPECT_EQ(0u, decode_token(-3.0));
  EXPECT_EQ(0u, decode_token(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_EQ(0u, decode_token(1e300));
}

TEST_F(TraceTest, a_running_chain_passes) { EXPECT_EQ(std::nullopt, audit(chain())); }

TEST_F(TraceTest, a_departed_writer_is_found_through_the_provider)
{
  const uint64_t token = chain();
  trace_.set_controller_state(controller("wrist"), Lifecycle::Inactive);
  const auto event = trace_.audit(token, interface("a3/velocity"), ARM);
  ASSERT_TRUE(event.has_value());
  EXPECT_EQ(EventKind::InactiveWriter, event->kind);
  EXPECT_EQ(controller("wrist"), event->writer);
}

TEST_F(TraceTest, a_restarted_writer_does_not_own_its_old_tokens)
{
  const uint64_t token = chain();
  trace_.set_controller_state(controller("arm_vel"), Lifecycle::Inactive);
  trace_.set_controller_state(controller("arm_vel"), Lifecycle::Active);
  EXPECT_EQ(EventKind::InactiveWriter, audit(token));
}

TEST_F(TraceTest, async_hardware_may_apply_a_command_one_cycle_after_its_writer_stopped)
{
  const uint64_t token = chain();
  trace_.set_controller_state(controller("wrist"), Lifecycle::Inactive);
  const auto subject = ARM;
  EXPECT_FALSE(
    trace_.audit(token, interface("a3/velocity"), subject, Applier::AsyncThread).has_value());
  EXPECT_TRUE(trace_.audit(token, interface("a3/velocity"), subject).has_value());
  trace_.begin_cycle();
  trace_.begin_cycle();
  EXPECT_TRUE(
    trace_.audit(token, interface("a3/velocity"), subject, Applier::AsyncThread).has_value());
}

TEST_F(TraceTest, without_grace_async_hardware_must_not_apply_a_stopped_writers_command)
{
  const auto applied_after_stop = [&]
  {
    const uint64_t token = chain();
    trace_.set_controller_state(controller("wrist"), Lifecycle::Inactive);
    const auto event = trace_.audit(token, interface("a3/velocity"), ARM, Applier::AsyncThread);
    return event ? std::optional<EventKind>(event->kind) : std::nullopt;
  };
  trace_.set_async_grace_cycles(0);
  EXPECT_EQ(EventKind::InactiveWriter, applied_after_stop());
  trace_.reset();
  SetUp();
  EXPECT_EQ(EventKind::InactiveWriter, applied_after_stop()) << "the grace survives a reset";
}

TEST_F(TraceTest, a_failure_belongs_to_one_activation)
{
  const auto arm_vel = controller("arm_vel");
  trace_.set_controller_failed(arm_vel);
  EXPECT_EQ(trace_.controller(arm_vel).epoch, trace_.controller(arm_vel).failed_epoch);
  trace_.set_controller_state(arm_vel, Lifecycle::Inactive);
  trace_.set_controller_state(arm_vel, Lifecycle::Active);
  EXPECT_NE(trace_.controller(arm_vel).epoch, trace_.controller(arm_vel).failed_epoch);
}

TEST_F(TraceTest, a_refused_stop_counts_as_stopped)
{
  const auto arm_vel = controller("arm_vel");
  const uint64_t token = trace_.mint(arm_vel, interface("a1/velocity"), 0);
  trace_.set_controller_refused_stop(arm_vel);
  EXPECT_EQ(Lifecycle::Inactive, trace_.controller(arm_vel).state);
  EXPECT_TRUE(trace_.controller(arm_vel).refused_stop);
  EXPECT_TRUE(trace_.audit(token, interface("a1/velocity"), ARM).has_value());
}

TEST_F(TraceTest, a_held_read_is_reported_once_per_hardware)
{
  trace_.hold_reads(std::chrono::steady_clock::now() + std::chrono::milliseconds(5));
  EXPECT_TRUE(trace_.wait_while_held());
  EXPECT_FALSE(trace_.wait_while_held());
  trace_.mark_held(0);
  trace_.mark_held(0);
  EXPECT_EQ(std::vector<uint16_t>{0}, trace_.take_held());
  EXPECT_TRUE(trace_.take_held().empty());
}

TEST_F(TraceTest, old_tokens_are_stale)
{
  const uint64_t token = chain();
  for (uint64_t i = 0; i <= MAX_COMMAND_AGE_CYCLES; ++i)
  {
    trace_.begin_cycle();
  }
  EXPECT_EQ(EventKind::StaleValue, audit(token));
}

TEST_F(TraceTest, async_hardware_measures_the_age_on_the_wall_clock)
{
  const uint64_t token = chain();
  for (uint64_t i = 0; i <= MAX_COMMAND_AGE_CYCLES; ++i)
  {
    trace_.begin_cycle();
  }
  const auto age = [&]
  {
    const auto event = trace_.audit(token, interface("a3/velocity"), ARM, Applier::AsyncThread);
    return event ? std::optional<EventKind>(event->kind) : std::nullopt;
  };
  EXPECT_EQ(std::nullopt, age());
  std::this_thread::sleep_for(MAX_COMMAND_AGE + std::chrono::milliseconds(1));
  EXPECT_EQ(EventKind::StaleValue, age());
}

TEST_F(TraceTest, async_hardware_ages_a_token_until_it_was_applied)
{
  const uint64_t token = chain();
  for (uint64_t i = 0; i <= MAX_COMMAND_AGE_CYCLES; ++i)
  {
    trace_.begin_cycle();
  }
  const auto age = [&](std::chrono::steady_clock::time_point applied_at)
  {
    const auto event =
      trace_.audit(token, interface("a3/velocity"), ARM, Applier::AsyncThread, {}, applied_at);
    return event ? std::optional<EventKind>(event->kind) : std::nullopt;
  };
  const auto applied_in_time = std::chrono::steady_clock::now();
  std::this_thread::sleep_for(MAX_COMMAND_AGE + std::chrono::milliseconds(1));
  EXPECT_EQ(std::nullopt, age(applied_in_time)) << "the wait after the apply is not age";
  EXPECT_EQ(EventKind::StaleValue, age(std::chrono::steady_clock::now()));
}

TEST_F(TraceTest, a_token_in_the_wrong_interface_is_misrouted)
{
  const uint64_t token = trace_.mint(controller("arm_vel"), interface("a2/velocity"), 0);
  EXPECT_EQ(EventKind::MisroutedValue, audit(token));
}

TEST_F(TraceTest, unknown_values_are_reported)
{
  EXPECT_EQ(EventKind::UnknownValue, audit(0));
  EXPECT_EQ(EventKind::UnknownValue, audit(decode_token(encode_token(12345))));
}

TEST_F(TraceTest, an_overwritten_token_stays_attributable)
{
  const uint64_t token = chain();
  for (size_t i = 0; i < Trace::TOKEN_CAPACITY; ++i)
  {
    trace_.mint(controller("arm_vel"), interface("a1/velocity"), 0);
  }
  // Older than the ring, so stale while its writer still runs in the same activation
  EXPECT_EQ(EventKind::StaleValue, audit(token));
  // A stopped writer's old command is the late start bug class, an inactive writer
  trace_.set_controller_state(controller("arm_vel"), Lifecycle::Inactive);
  EXPECT_EQ(EventKind::InactiveWriter, audit(token));
}

TEST_F(TraceTest, a_token_from_before_the_reset_is_stale_on_async_hardware)
{
  const uint64_t token = chain();
  trace_.reset();
  SetUp();
  const auto event =
    trace_.audit(token, interface("a3/velocity"), ARM, Applier::AsyncThread, std::chrono::hours(1));
  EXPECT_EQ(EventKind::StaleValue, event ? std::optional(event->kind) : std::nullopt);
}

TEST_F(TraceTest, an_overwritten_token_ages_on_the_wall_clock_on_async_hardware)
{
  const uint64_t token = chain();
  overwrite_the_ring();
  const auto age = [&](std::chrono::nanoseconds preempted)
  {
    const auto event =
      trace_.audit(token, interface("a3/velocity"), ARM, Applier::AsyncThread, preempted);
    return event ? std::optional<EventKind>(event->kind) : std::nullopt;
  };
  std::this_thread::sleep_for(MAX_COMMAND_AGE + std::chrono::milliseconds(1));
  EXPECT_EQ(std::nullopt, age(std::chrono::hours(1))) << "its applier waited preempted";
  EXPECT_EQ(EventKind::StaleValue, age({}));
}

TEST_F(TraceTest, an_owner_that_keeps_applying_an_old_copy_is_caught_after_the_ring_wrapped)
{
  const uint64_t copy = chain();
  overwrite_the_ring();
  // The slot mate of the copy is renewed every lap, half a lap later it is at least twice
  // MAX_COMMAND_AGE old
  constexpr int LAP = Trace::TOKEN_CAPACITY / 1024;
  for (int i = 0; i < 3 * LAP; ++i)
  {
    for (int token = 0; token < 1024; ++token)
    {
      trace_.mint(controller("arm_vel"), interface("a1/velocity"), 0);
    }
    trace_.begin_cycle();
    std::this_thread::sleep_for(std::chrono::microseconds(4 * MAX_COMMAND_AGE) / LAP);
    const auto event = trace_.audit(copy, interface("a3/velocity"), ARM, Applier::AsyncThread);
    if (i % LAP == LAP / 2)
    {
      EXPECT_EQ(EventKind::StaleValue, event ? std::optional(event->kind) : std::nullopt)
        << "iteration " << i;
    }
  }
}

TEST_F(TraceTest, async_hardware_excuses_the_control_loop_preempted_in_the_minting_cycle_only)
{
  trace_.begin_cycle();
  const uint64_t delayed = chain();
  trace_.end_cycle(2 * MAX_COMMAND_AGE);
  trace_.begin_cycle();
  const uint64_t punctual = chain();
  trace_.end_cycle({});
  for (uint64_t i = 0; i <= MAX_COMMAND_AGE_CYCLES; ++i)
  {
    trace_.begin_cycle();
    trace_.end_cycle({});
  }
  const auto age = [&](uint64_t token)
  {
    const auto event = trace_.audit(token, interface("a3/velocity"), ARM, Applier::AsyncThread);
    return event ? std::optional<EventKind>(event->kind) : std::nullopt;
  };
  std::this_thread::sleep_for(MAX_COMMAND_AGE + std::chrono::milliseconds(1));
  EXPECT_EQ(std::nullopt, age(delayed));
  EXPECT_EQ(EventKind::StaleValue, age(punctual));
  std::this_thread::sleep_for(2 * MAX_COMMAND_AGE);
  EXPECT_EQ(EventKind::StaleValue, age(delayed)) << "the excuse must not grow with the age";
}

TEST_F(TraceTest, a_full_event_buffer_keeps_the_first_event_of_every_kind)
{
  for (size_t i = 0; i < Trace::EVENT_CAPACITY + 10; ++i)
  {
    trace_.report({EventKind::StaleValue, ARM, NO_INDEX, NO_INDEX, 0});
  }
  trace_.report({EventKind::InactiveWriter, ARM, NO_INDEX, NO_INDEX, 0});
  const auto events = trace_.take_events();
  EXPECT_TRUE(
    std::any_of(
      events.begin(), events.end(),
      [](const Event & e) { return e.kind == EventKind::InactiveWriter; }));
  EXPECT_EQ(EventKind::EventsLost, events.back().kind);
}

TEST_F(TraceTest, events_are_taken_once)
{
  trace_.report({EventKind::ModeConflict, ARM, NO_INDEX, NO_INDEX, 0});
  EXPECT_EQ(1u, trace_.take_events().size());
  EXPECT_TRUE(trace_.take_events().empty());
}

TEST_F(TraceTest, visited_states_include_transient_ones)
{
  const auto arm_vel = controller("arm_vel");
  trace_.take_visited_states(arm_vel);
  trace_.set_controller_state(arm_vel, Lifecycle::Inactive);
  trace_.set_controller_state(arm_vel, Lifecycle::Active);
  const uint32_t visited = trace_.take_visited_states(arm_vel);
  EXPECT_TRUE(visited & (1u << static_cast<uint32_t>(Lifecycle::Inactive)));
  EXPECT_FALSE(visited & (1u << static_cast<uint32_t>(Lifecycle::Unconfigured)));
}

TEST_F(TraceTest, slow_io_paces_async_hardware_until_turned_off_or_reset)
{
  using Clock = std::chrono::steady_clock;
  const auto paced = [&](bool write = false)
  {
    const auto start = Clock::now();
    trace_.pace_async_io(write);
    return Clock::now() - start;
  };
  EXPECT_LT(paced(), SLOW_IO);
  trace_.set_slow_io(SlowIo::Steady);
  EXPECT_GE(paced(), SLOW_IO);
  trace_.set_slow_io(SlowIo::Off);
  EXPECT_LT(paced(), SLOW_IO);
  trace_.set_slow_io(SlowIo::Steady);
  trace_.reset();
  EXPECT_LT(paced(), SLOW_IO);
}

TEST_F(TraceTest, a_stall_holds_only_writes)
{
  using Clock = std::chrono::steady_clock;
  trace_.set_slow_io(SlowIo::Stall);
  auto start = Clock::now();
  trace_.pace_async_io(false);
  EXPECT_LT(Clock::now() - start, IO_STALL);
  start = Clock::now();
  trace_.pace_async_io(true);
  EXPECT_GE(Clock::now() - start, IO_STALL);
}

TEST_F(TraceTest, a_long_stall_holds_writes_just_below_the_hang_rule)
{
  using Clock = std::chrono::steady_clock;
  trace_.set_slow_io(SlowIo::LongStall);
  auto start = Clock::now();
  trace_.pace_async_io(false);
  EXPECT_LT(Clock::now() - start, IO_STALL);
  start = Clock::now();
  trace_.pace_async_io(true);
  EXPECT_GE(Clock::now() - start, IO_LONG_STALL);
}

TEST_F(TraceTest, jitter_varies_up_to_its_bound)
{
  trace_.set_slow_io(SlowIo::Jitter);
  std::chrono::microseconds shortest = std::chrono::microseconds::max();
  std::chrono::microseconds longest{};
  for (int i = 0; i < 1000; ++i)
  {
    const auto delay = trace_.async_io_delay(i % 2 == 0);
    shortest = std::min(shortest, delay);
    longest = std::max(longest, delay);
  }
  EXPECT_LE(longest, IO_JITTER);
  EXPECT_LT(shortest, IO_JITTER / 4);
  EXPECT_GT(longest, IO_JITTER * 3 / 4);
}

TEST_F(TraceTest, jitter_repeats_after_a_reset)
{
  const auto delays = [this]
  {
    trace_.reset();
    trace_.set_slow_io(SlowIo::Jitter);
    std::vector<std::chrono::microseconds> drawn;
    for (int i = 0; i < 20; ++i)
    {
      drawn.push_back(trace_.async_io_delay(false));
    }
    return drawn;
  };
  EXPECT_EQ(delays(), delays());
}

TEST(Faults, are_taken_once_and_must_be_applicable)
{
  FaultBoard board;
  const Target arm{TargetKind::Hardware, 0};
  board.arm(arm, Hook::Write, Fault::Error);
  EXPECT_EQ(Fault::None, board.take(arm, Hook::Read));
  EXPECT_EQ(Fault::Error, board.take(arm, Hook::Write));
  EXPECT_EQ(Fault::None, board.take(arm, Hook::Write));
  EXPECT_THROW(board.arm(arm, Hook::Update, Fault::Error), std::invalid_argument);
  EXPECT_THROW(board.arm(arm, Hook::Deactivate, Fault::Failure), std::invalid_argument);
  EXPECT_NO_THROW(board.arm({TargetKind::Controller, 0}, Hook::Deactivate, Fault::Failure));
  EXPECT_NO_THROW(board.arm(arm, Hook::Perform, Fault::Delay));
}

TEST(Faults, a_quiet_activation_is_no_failure)
{
  FaultBoard board;
  const Target jsb{TargetKind::Controller, 0};
  EXPECT_THROW(
    board.arm({TargetKind::Hardware, 0}, Hook::Activate, Fault::Quiet), std::invalid_argument);
  EXPECT_THROW(board.arm(jsb, Hook::Configure, Fault::Quiet), std::invalid_argument);
  board.arm(jsb, Hook::Activate, Fault::Quiet);
  EXPECT_EQ(Fault::Quiet, board.take(jsb, Hook::Activate));
  EXPECT_TRUE(board.take_fired().empty());
}

TEST(Faults, late_callbacks_end_just_before_the_switch_deadline)
{
  FaultBoard board;
  const Target jsb{TargetKind::Controller, 0};
  const Target arm{TargetKind::Hardware, 0};
  EXPECT_NO_THROW(board.arm(jsb, Hook::Activate, Fault::Late));
  EXPECT_NO_THROW(board.arm(jsb, Hook::Deactivate, Fault::Late));
  EXPECT_NO_THROW(board.arm(arm, Hook::Error, Fault::Late));
  EXPECT_THROW(board.arm(jsb, Hook::Configure, Fault::Late), std::invalid_argument);
  EXPECT_THROW(board.arm(arm, Hook::Perform, Fault::Late), std::invalid_argument);
  const auto timeout = CmConfig{}.switch_timeout;
  EXPECT_LT(LATE, timeout);
  EXPECT_GT(LATE + SLOW_SWITCH, timeout) << "the deadline must pass in the slow call after it";
}

TEST(Faults, report_which_targets_fired)
{
  FaultBoard board;
  const Target arm{TargetKind::Hardware, 0};
  const Target jsb{TargetKind::Controller, 0};
  board.arm(arm, Hook::Write, Fault::Error);
  board.arm(jsb, Hook::Update, Fault::Error);
  board.take(arm, Hook::Write);
  const auto fired = board.take_fired();
  ASSERT_EQ(1u, fired.size());
  EXPECT_EQ(TargetKind::Hardware, fired[0].kind);
  EXPECT_TRUE(board.take_fired().empty());
}

}  // namespace cm_test_suite
