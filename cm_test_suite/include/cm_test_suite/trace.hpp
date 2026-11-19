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

#ifndef CM_TEST_SUITE__TRACE_HPP_
#define CM_TEST_SUITE__TRACE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "cm_test_suite/types.hpp"

/// Ground truth shared between the rig plugins and the harness.
///
/// The plugins record what they experience (their own lifecycle, the tokens they mint, the
/// commands their hardware applies) independent of what the controller manager reports. All
/// members are safe to use from the real-time thread, async hardware threads and the harness.
namespace cm_test_suite
{
/// Problems the plugins detect at the moment they happen.
enum class EventKind : uint8_t
{
  // A value found in an interface: a command at hardware write, a reference or exported state
  // read by a controller.
  InactiveWriter,
  StaleValue,
  UnknownValue,
  MisroutedValue,
  /// A command outside the limits of its joint in the robot description.
  LimitExceeded,
  /// One write applies commands of one activation of a controller from different updates.
  TornCommand,
  // Hardware.
  ModeConflict,
  IoWhileInactive,
  IoAfterFault,
  ConcurrentHardwareCall,
  // Controllers. Subject is the controller.
  UpdateWhileInactive,
  MissingInterfaces,
  ActivatedBeforeProvider,
  ActivatedOnUnchainedProvider,
  ActivatedOnInactiveHardware,
  ProviderStoppedUnderUser,
  UpdateAfterFailure,
  ConcurrentControllerCall,
  /// Activated while another active controller claims one of its command or reference interfaces
  /// or another mode of a joint it commands. Writer is the other controller.
  SharedCommandInterface,
  // More events than the trace can hold.
  EventsLost,
};

const char * to_string(EventKind kind);
/// Snake case name, used as check category.
const char * slug(EventKind kind);

struct Event
{
  EventKind kind;
  Target subject;
  /// Interface id, or NO_INDEX.
  uint16_t interface;
  /// Controller that wrote the offending value, or NO_INDEX.
  uint16_t writer;
  uint64_t cycle;
};

constexpr uint16_t NO_INDEX = UINT16_MAX;

struct TokenRecord
{
  uint16_t controller;
  uint16_t interface;
  uint32_t epoch;
  uint64_t parent;
  uint64_t cycle;
  std::chrono::steady_clock::time_point minted;
};

struct ControllerTruth
{
  Lifecycle state;
  /// Counts activations, so a restarted controller is distinguishable from a running one.
  uint32_t epoch;
  bool chained;
  /// The epoch whose update failed, 0 if none did.
  uint32_t failed_epoch;
  /// Its on_deactivate refused, so it counts as inactive although its lifecycle is still active.
  bool refused_stop;
  uint64_t updates;
};

struct HardwareTruth
{
  Lifecycle state;
  /// A read or write failed and no lifecycle transition followed yet.
  bool faulted;
  uint64_t reads;
  uint64_t writes;
};

struct InterfaceTruth
{
  /// The hardware applies commands of this interface, it is started and not held.
  bool live;
  /// Last cycle a command token was applied, 0 if never.
  uint64_t applied_cycle;
};

/// Values older than this many cycles count as stale, even if their writer still runs. A pause of a
/// running controller beyond it requires the hardware to hold.
constexpr uint64_t MAX_COMMAND_AGE_CYCLES = 5;

/// Async hardware may apply data latched one cycle earlier, so its writer may have stopped since.
/// A controller manager that holds an async component before its writer stops needs none.
constexpr uint64_t DEFAULT_ASYNC_GRACE_CYCLES = 1;

/// How async hardware paces its read and write in a slow I/O step.
enum class SlowIo : uint8_t
{
  Off,
  /// Every read and write takes SLOW_IO.
  Steady,
  /// Every read and write takes a random time up to IO_JITTER.
  Jitter,
  /// Every write takes IO_STALL, an owner thread stalled below the hang rule.
  Stall,
  /// Every write takes IO_LONG_STALL, just below the hang rule.
  LongStall,
};

/// MAX_COMMAND_AGE_CYCLES at the 1 ms period of the control loop. Async hardware runs on its own
/// clock, so in stepped mode, where cycles take microseconds, its age counts on the wall clock.
/// Sanitizers slow every thread about tenfold, like the runner's cycle limit it scales.
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
constexpr std::chrono::milliseconds MAX_COMMAND_AGE{50};
#else
constexpr std::chrono::milliseconds MAX_COMMAND_AGE{5};
#endif

/// Who applies a command: the control loop, or the own thread of async hardware.
enum class Applier
{
  ControlLoop,
  AsyncThread,
};

/// How long the calling thread has waited runnable on a run queue, the time others preempted it.
std::chrono::nanoseconds preempted_time();

/// A command value is the token as a double, exact below 2^53. NaN means no command. A token packs
/// a running id with the writer and its epoch, so it stays attributable after its record was
/// overwritten in the token ring.
double encode_token(uint64_t token);
/// Returns 0 for values that are not a token.
uint64_t decode_token(double value);

class Trace
{
public:
  static constexpr size_t MAX_CONTROLLERS = 32;
  static constexpr size_t MAX_HARDWARE = 8;
  static constexpr size_t MAX_INTERFACES = 64;
  static constexpr size_t TOKEN_CAPACITY = 1 << 16;
  static constexpr size_t CYCLE_CAPACITY = 1 << 16;
  static constexpr size_t EVENT_CAPACITY = 4096;
  /// Room for the first event of every kind beyond the capacity, so reporting never allocates.
  static constexpr size_t EVENT_RESERVE =
    EVENT_CAPACITY + static_cast<size_t>(EventKind::EventsLost) + 1;
  static constexpr int MAX_CHAIN_DEPTH = 8;

  Trace();

  /// Forget everything. Only call while no controller manager exists.
  void reset();

  void begin_cycle();
  /// The control loop waited this long on a run queue in the current cycle, which delays the
  /// handoff of the cycle's commands to async hardware without aging them.
  void end_cycle(std::chrono::nanoseconds preempted);
  uint64_t cycle() const;

  void set_power(bool on);
  bool power() const;

  void set_slow_io(SlowIo pace);
  /// Called by async hardware at the end of read and write. Takes as long as the pace says.
  void pace_async_io(bool write) const;
  /// How long the pace holds one read or write, random with jitter.
  std::chrono::microseconds async_io_delay(bool write) const;

  /// Survives reset, the suite sets it once per process.
  void set_async_grace_cycles(uint64_t cycles);

  /// Holds the control loop inside the read of sync hardware until released or until the deadline.
  void hold_reads(std::chrono::steady_clock::time_point until);
  void release_reads();
  /// Called by sync hardware in read. Returns whether it was held.
  bool wait_while_held() const;
  /// A held read is a hung hardware call (rulebook 2.2.6), so it excuses like a fault.
  void mark_held(uint16_t hardware);
  /// The hardware whose read was held since the last call.
  std::vector<uint16_t> take_held();

  void set_controller_state(uint16_t controller, Lifecycle state);
  void set_controller_chained(uint16_t controller, bool chained);
  /// The update of the current epoch failed.
  void set_controller_failed(uint16_t controller);
  /// on_deactivate refused. The controller counts as inactive from now on.
  void set_controller_refused_stop(uint16_t controller);
  void count_update(uint16_t controller);
  ControllerTruth controller(uint16_t controller) const;
  /// Bit set of the states a controller passed through since the last call.
  uint32_t take_visited_states(uint16_t controller);

  void set_hardware_state(uint16_t hardware, Lifecycle state);
  void set_hardware_faulted(uint16_t hardware, bool faulted);
  void count_read(uint16_t hardware);
  void count_write(uint16_t hardware);
  HardwareTruth hardware(uint16_t hardware) const;
  uint32_t take_visited_hardware_states(uint16_t hardware);

  void set_interface_live(uint16_t interface, bool live);
  void set_interface_applied(uint16_t interface);
  InterfaceTruth interface(uint16_t interface) const;

  /// Record a command the controller writes into an interface, derived from parent (0 for none).
  uint64_t mint(uint16_t controller, uint16_t interface, uint64_t parent);
  std::optional<TokenRecord> lookup(uint64_t token) const;

  /// Check a token found in an interface. Returns the first problem along its chain. An async
  /// applier passes the time it was preempted since its last write, which is not age. That span
  /// may begin before the token was minted, like the control loop's wait in the minting cycle.
  /// It ages the token until applied_at, taken before the preemption so no wait falls in between.
  std::optional<Event> audit(
    uint64_t token, uint16_t interface, Target subject, Applier applier = Applier::ControlLoop,
    std::chrono::nanoseconds preempted = {},
    std::chrono::steady_clock::time_point applied_at = std::chrono::steady_clock::now()) const;

  void begin_hang(Target target);
  void end_hang(Target target);
  /// Injected hangs still sleeping in a plugin callback.
  int hangs_running() const;
  /// Whether an injected hang sleeps in a callback of this controller.
  bool controller_hanging(uint16_t controller) const;
  bool hardware_hanging(uint16_t hardware) const;

  /// Lifecycle and mode switch callbacks that run right now, so a step is not settled meanwhile.
  void begin_call();
  void end_call();
  int calls_running() const;

  void report(const Event & event);
  std::vector<Event> take_events();

private:
  std::chrono::nanoseconds cycle_preempted(uint64_t cycle) const;
  /// The record in the token's slot, with any_token also one of a younger token.
  std::optional<TokenRecord> read_slot(uint64_t token, bool any_token) const;

  struct ControllerSlot
  {
    std::atomic<Lifecycle> state{Lifecycle::Unloaded};
    std::atomic<uint32_t> epoch{0};
    std::atomic<bool> chained{false};
    std::atomic<uint32_t> failed_epoch{0};
    std::atomic<bool> refused_stop{false};
    std::atomic<uint64_t> updates{0};
    std::atomic<uint64_t> left_active_cycle{0};
    std::atomic<uint32_t> visited{0};
    std::atomic<int> hanging{0};
  };

  struct HardwareSlot
  {
    std::atomic<Lifecycle> state{Lifecycle::Unloaded};
    std::atomic<bool> faulted{false};
    std::atomic<uint64_t> reads{0};
    std::atomic<uint64_t> writes{0};
    std::atomic<uint32_t> visited{0};
    std::atomic<int> hanging{0};
  };

  struct InterfaceSlot
  {
    std::atomic<bool> live{false};
    std::atomic<uint64_t> applied_cycle{0};
  };

  // Seqlock entry, the id is cleared while the fields change
  struct TokenSlot
  {
    std::atomic<uint64_t> id{0};
    std::atomic<uint16_t> controller{0};
    std::atomic<uint16_t> interface{0};
    std::atomic<uint32_t> epoch{0};
    std::atomic<uint64_t> parent{0};
    std::atomic<uint64_t> cycle{0};
    std::atomic<std::chrono::steady_clock::rep> minted{0};
  };

  std::atomic<uint64_t> cycle_{0};
  std::atomic<bool> power_{true};
  std::atomic<SlowIo> slow_io_{SlowIo::Off};
  mutable std::atomic<uint64_t> io_calls_{0};
  std::atomic<uint64_t> async_grace_cycles_{DEFAULT_ASYNC_GRACE_CYCLES};
  std::atomic<std::chrono::steady_clock::rep> held_until_{0};
  std::atomic<uint32_t> held_hardware_{0};
  std::atomic<uint64_t> next_token_{1};
  std::atomic<int> hangs_running_{0};
  std::atomic<int> calls_running_{0};
  std::array<ControllerSlot, MAX_CONTROLLERS> controllers_;
  std::array<HardwareSlot, MAX_HARDWARE> hardware_;
  std::array<InterfaceSlot, MAX_INTERFACES> interfaces_;
  std::vector<TokenSlot> tokens_;
  std::vector<std::atomic<std::chrono::nanoseconds::rep>> cycle_preempted_;

  std::mutex events_mutex_;
  std::vector<Event> events_;
  bool events_overflowed_ = false;
};

/// The process wide trace the rig plugins write to.
Trace & trace();

}  // namespace cm_test_suite

#endif  // CM_TEST_SUITE__TRACE_HPP_
