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

#include "cm_test_suite/trace.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <thread>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <utility>

#include "cm_test_suite/faults.hpp"

namespace cm_test_suite
{
namespace
{
constexpr uint64_t MAX_EXACT_INTEGER = uint64_t{1} << 53;
// Token layout: 32 bit id, 5 bit controller, 16 bit epoch, 53 bits in all
constexpr int CONTROLLER_SHIFT = 32;
constexpr int EPOCH_SHIFT = 37;
constexpr uint64_t ID_MASK = (uint64_t{1} << CONTROLLER_SHIFT) - 1;
constexpr uint64_t CONTROLLER_MASK = (uint64_t{1} << (EPOCH_SHIFT - CONTROLLER_SHIFT)) - 1;
constexpr uint64_t EPOCH_MASK = (uint64_t{1} << 16) - 1;
static_assert(Trace::MAX_CONTROLLERS <= CONTROLLER_MASK + 1);
static_assert(EPOCH_SHIFT + 16 <= 53);

uint32_t state_bit(Lifecycle state) { return uint32_t{1} << static_cast<uint32_t>(state); }
}  // namespace

const char * to_string(Lifecycle state)
{
  switch (state)
  {
    case Lifecycle::Unloaded:
      return "unloaded";
    case Lifecycle::Unconfigured:
      return "unconfigured";
    case Lifecycle::Inactive:
      return "inactive";
    case Lifecycle::Active:
      return "active";
    case Lifecycle::Finalized:
      return "finalized";
  }
  return "invalid";
}

namespace
{
struct EventText
{
  const char * slug;
  const char * description;
};

constexpr std::array<EventText, static_cast<size_t>(EventKind::EventsLost) + 1> EVENT_TEXTS = {{
  {"inactive_writer", "value written by an inactive or restarted controller"},
  {"stale_value", "value too old"},
  {"unknown_value", "value is no known token"},
  {"misrouted_value", "value minted for another interface"},
  {"limit_exceeded", "command outside the limits of its joint"},
  {"torn_command", "commands of one controller from different updates in one write"},
  {"mode_conflict", "two command modes active on one joint"},
  {"io_while_inactive", "write of inactive or read of unconfigured hardware"},
  {"io_after_fault", "read or write after a fault before recovery"},
  {"concurrent_hardware_call", "concurrent calls into one hardware component"},
  {"update_while_inactive", "update of an inactive controller"},
  {"missing_interfaces", "active controller without its interfaces"},
  {"activated_before_provider", "activated while a provider is inactive"},
  {"activated_on_unchained_provider", "activated while a provider is not in chained mode"},
  {"activated_on_inactive_hardware", "activated while hardware it uses is not available"},
  {"provider_stopped_under_user", "deactivated while a user is active"},
  {"update_after_failure", "updated again after its update failed, without a restart"},
  {"concurrent_controller_call", "concurrent calls into one controller"},
  {"shared_command_interface",
   "activated while another active controller claims its command interfaces or joints"},
  {"events_lost", "events lost"},
}};
}  // namespace

const char * to_string(EventKind kind)
{
  return EVENT_TEXTS.at(static_cast<size_t>(kind)).description;
}

const char * slug(EventKind kind) { return EVENT_TEXTS.at(static_cast<size_t>(kind)).slug; }

std::chrono::nanoseconds preempted_time()
{
  // Plain syscalls, as stdio streams take a process wide lock that a preempted holder keeps
  std::array<char, 128> text{};
  const int fd = open("/proc/thread-self/schedstat", O_RDONLY | O_CLOEXEC);
  const ssize_t length = fd < 0 ? -1 : read(fd, text.data(), text.size() - 1);
  if (fd >= 0)
  {
    close(fd);
  }
  unsigned long long on_cpu = 0;
  unsigned long long run_queue_wait = 0;
  if (length <= 0 || std::sscanf(text.data(), "%llu %llu", &on_cpu, &run_queue_wait) != 2)
  {
    throw std::runtime_error("Cannot read /proc/thread-self/schedstat");
  }
  return std::chrono::nanoseconds(run_queue_wait);
}

double encode_token(uint64_t token) { return static_cast<double>(token); }

uint64_t decode_token(double value)
{
  if (!std::isfinite(value) || value < 1.0 || value >= static_cast<double>(MAX_EXACT_INTEGER))
  {
    return 0;
  }
  const auto token = static_cast<uint64_t>(value);
  return static_cast<double>(token) == value ? token : 0;
}

Trace::Trace() : tokens_(TOKEN_CAPACITY), cycle_preempted_(CYCLE_CAPACITY)
{
  events_.reserve(EVENT_RESERVE);
}

void Trace::reset()
{
  cycle_ = 0;
  power_ = true;
  slow_io_ = SlowIo::Off;
  io_calls_ = 0;
  held_until_ = 0;
  held_hardware_ = 0;
  for (auto & slot : controllers_)
  {
    slot.state = Lifecycle::Unloaded;
    slot.epoch = 0;
    slot.chained = false;
    slot.failed_epoch = 0;
    slot.refused_stop = false;
    slot.updates = 0;
    slot.left_active_cycle = 0;
    slot.visited = 0;
    slot.hanging = 0;
  }
  for (auto & slot : hardware_)
  {
    slot.state = Lifecycle::Unloaded;
    slot.faulted = false;
    slot.reads = 0;
    slot.writes = 0;
    slot.visited = 0;
    slot.hanging = 0;
  }
  for (auto & slot : interfaces_)
  {
    slot.live = false;
    slot.applied_cycle = 0;
  }
  for (auto & slot : tokens_)
  {
    slot.id = 0;
  }
  for (auto & preempted : cycle_preempted_)
  {
    preempted = 0;
  }
  // Tokens keep growing across runs, so a value left in a handle can never match a new record
  std::lock_guard<std::mutex> lock(events_mutex_);
  events_.clear();
  events_overflowed_ = false;
}

void Trace::begin_cycle() { cycle_.fetch_add(1); }

void Trace::end_cycle(std::chrono::nanoseconds preempted)
{
  cycle_preempted_[cycle_.load() % CYCLE_CAPACITY] = preempted.count();
}

uint64_t Trace::cycle() const { return cycle_.load(); }

std::chrono::nanoseconds Trace::cycle_preempted(uint64_t cycle) const
{
  const std::chrono::nanoseconds preempted(cycle_preempted_[cycle % CYCLE_CAPACITY].load());
  // Read before the check, so a later cycle cannot have overwritten it unnoticed
  return cycle_.load() < cycle + CYCLE_CAPACITY ? preempted : std::chrono::nanoseconds{};
}

void Trace::set_power(bool on) { power_ = on; }

bool Trace::power() const { return power_; }

void Trace::set_slow_io(SlowIo pace) { slow_io_ = pace; }

std::chrono::microseconds Trace::async_io_delay(bool write) const
{
  using std::chrono::microseconds;
  const SlowIo pace = slow_io_.load();
  switch (pace)
  {
    case SlowIo::Off:
      return {};
    case SlowIo::Steady:
      return SLOW_IO;
    case SlowIo::Jitter:
    {
      // splitmix64 of a call counter, a fixed sequence dealt out in scheduling order
      uint64_t x = io_calls_.fetch_add(1) + 0x9e3779b97f4a7c15;
      x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9;
      x = (x ^ (x >> 27)) * 0x94d049bb133111eb;
      x ^= x >> 31;
      return microseconds(static_cast<int64_t>(x % (microseconds(IO_JITTER).count() + 1)));
    }
    case SlowIo::Stall:
      return write ? IO_STALL : microseconds{};
    case SlowIo::LongStall:
      return write ? IO_LONG_STALL : microseconds{};
  }
  return {};
}

void Trace::pace_async_io(bool write) const
{
  if (const auto delay = async_io_delay(write); delay.count() > 0)
  {
    std::this_thread::sleep_for(delay);
  }
}

void Trace::set_async_grace_cycles(uint64_t cycles) { async_grace_cycles_ = cycles; }

void Trace::hold_reads(std::chrono::steady_clock::time_point until)
{
  held_until_ = until.time_since_epoch().count();
}

void Trace::release_reads() { held_until_ = 0; }

void Trace::mark_held(uint16_t hardware) { held_hardware_.fetch_or(uint32_t{1} << hardware); }

std::vector<uint16_t> Trace::take_held()
{
  const uint32_t mask = held_hardware_.exchange(0);
  std::vector<uint16_t> held;
  for (uint16_t i = 0; i < 32; ++i)
  {
    if ((mask & (uint32_t{1} << i)) != 0)
    {
      held.push_back(i);
    }
  }
  return held;
}

bool Trace::wait_while_held() const
{
  bool held = false;
  while (std::chrono::steady_clock::now().time_since_epoch().count() < held_until_.load())
  {
    held = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return held;
}

void Trace::set_controller_state(uint16_t controller, Lifecycle state)
{
  auto & slot = controllers_.at(controller);
  if (state == Lifecycle::Active)
  {
    slot.epoch.fetch_add(1);
    slot.refused_stop = false;
  }
  else if (slot.state == Lifecycle::Active)
  {
    slot.left_active_cycle = cycle_.load();
  }
  slot.state = state;
  slot.visited.fetch_or(state_bit(state));
}

void Trace::set_controller_chained(uint16_t controller, bool chained)
{
  controllers_.at(controller).chained = chained;
}

void Trace::set_controller_failed(uint16_t controller)
{
  auto & slot = controllers_.at(controller);
  slot.failed_epoch = slot.epoch.load();
}

void Trace::set_controller_refused_stop(uint16_t controller)
{
  set_controller_state(controller, Lifecycle::Inactive);
  controllers_.at(controller).refused_stop = true;
}

void Trace::count_update(uint16_t controller) { controllers_.at(controller).updates.fetch_add(1); }

ControllerTruth Trace::controller(uint16_t controller) const
{
  const auto & slot = controllers_.at(controller);
  return {slot.state.load(),        slot.epoch.load(),        slot.chained.load(),
          slot.failed_epoch.load(), slot.refused_stop.load(), slot.updates.load()};
}

uint32_t Trace::take_visited_states(uint16_t controller)
{
  auto & slot = controllers_.at(controller);
  return slot.visited.exchange(state_bit(slot.state.load()));
}

void Trace::set_hardware_state(uint16_t hardware, Lifecycle state)
{
  auto & slot = hardware_.at(hardware);
  slot.state = state;
  slot.visited.fetch_or(state_bit(state));
}

void Trace::set_hardware_faulted(uint16_t hardware, bool faulted)
{
  hardware_.at(hardware).faulted = faulted;
}

void Trace::count_read(uint16_t hardware) { hardware_.at(hardware).reads.fetch_add(1); }

void Trace::count_write(uint16_t hardware) { hardware_.at(hardware).writes.fetch_add(1); }

HardwareTruth Trace::hardware(uint16_t hardware) const
{
  const auto & slot = hardware_.at(hardware);
  return {slot.state.load(), slot.faulted.load(), slot.reads.load(), slot.writes.load()};
}

uint32_t Trace::take_visited_hardware_states(uint16_t hardware)
{
  auto & slot = hardware_.at(hardware);
  return slot.visited.exchange(state_bit(slot.state.load()));
}

void Trace::set_interface_live(uint16_t interface, bool live)
{
  interfaces_.at(interface).live = live;
}

void Trace::set_interface_applied(uint16_t interface)
{
  interfaces_.at(interface).applied_cycle = cycle_.load();
}

InterfaceTruth Trace::interface(uint16_t interface) const
{
  const auto & slot = interfaces_.at(interface);
  return {slot.live.load(), slot.applied_cycle.load()};
}

uint64_t Trace::mint(uint16_t controller, uint16_t interface, uint64_t parent)
{
  const uint64_t id = next_token_.fetch_add(1) & ID_MASK;
  const uint64_t epoch = controllers_.at(controller).epoch.load();
  const uint64_t token =
    id | (uint64_t{controller} << CONTROLLER_SHIFT) | ((epoch & EPOCH_MASK) << EPOCH_SHIFT);
  auto & slot = tokens_[id % TOKEN_CAPACITY];
  // A reader that sees any new field also sees the cleared id, so it never mixes two records
  slot.id.store(0, std::memory_order_relaxed);
  slot.controller.store(controller, std::memory_order_release);
  slot.interface.store(interface, std::memory_order_release);
  slot.epoch.store(static_cast<uint32_t>(epoch), std::memory_order_release);
  slot.parent.store(parent, std::memory_order_release);
  slot.cycle.store(cycle_.load(), std::memory_order_release);
  slot.minted.store(
    std::chrono::steady_clock::now().time_since_epoch().count(), std::memory_order_release);
  slot.id.store(token, std::memory_order_release);
  return token;
}

std::optional<TokenRecord> Trace::lookup(uint64_t token) const { return read_slot(token, false); }

std::optional<TokenRecord> Trace::read_slot(uint64_t token, bool any_token) const
{
  const auto & slot = tokens_[(token & ID_MASK) % TOKEN_CAPACITY];
  const uint64_t id = slot.id.load(std::memory_order_acquire);
  if (id == 0 || (!any_token && id != token))
  {
    return std::nullopt;
  }
  TokenRecord record{
    slot.controller.load(std::memory_order_acquire),
    slot.interface.load(std::memory_order_acquire),
    slot.epoch.load(std::memory_order_acquire),
    slot.parent.load(std::memory_order_acquire),
    slot.cycle.load(std::memory_order_acquire),
    std::chrono::steady_clock::time_point(
      std::chrono::steady_clock::duration(slot.minted.load(std::memory_order_acquire)))};
  if (slot.id.load(std::memory_order_relaxed) != id)
  {
    return std::nullopt;
  }
  return record;
}

std::optional<Event> Trace::audit(
  uint64_t token, uint16_t interface, Target subject, Applier applier,
  std::chrono::nanoseconds preempted, std::chrono::steady_clock::time_point applied_at) const
{
  const uint64_t now = cycle();
  const bool async = applier == Applier::AsyncThread;
  const uint64_t grace_cycles = async ? async_grace_cycles_.load() : 0;
  uint16_t expected_interface = interface;
  const auto too_old = [&](const TokenRecord & record)
  {
    return now > record.cycle + MAX_COMMAND_AGE_CYCLES &&
           (!async || applied_at - record.minted - preempted - cycle_preempted(record.cycle) >
                        MAX_COMMAND_AGE);
  };
  for (int depth = 0;; ++depth)
  {
    const auto record = token == 0 ? std::nullopt : lookup(token);
    if (!record && token != 0 && depth < MAX_CHAIN_DEPTH)
    {
      // Overwritten in the ring, so at least TOKEN_CAPACITY tokens old, but still attributable
      const auto writer_index =
        static_cast<uint16_t>((token >> CONTROLLER_SHIFT) & CONTROLLER_MASK);
      const uint64_t id = token & ID_MASK;
      const bool minted = id != 0 && id < (next_token_.load() & ID_MASK);
      if (minted && writer_index < controllers_.size())
      {
        const auto writer = controller(writer_index);
        const bool running = writer.state == Lifecycle::Active &&
                             (writer.epoch & EPOCH_MASK) == ((token >> EPOCH_SHIFT) & EPOCH_MASK);
        if (!running)
        {
          return Event{EventKind::InactiveWriter, subject, interface, writer_index, now};
        }
        // Far beyond MAX_COMMAND_AGE_CYCLES, but on the wall clock the younger token in its slot
        // only bounds its age from below. An empty slot means a token from before the reset
        const auto younger = async ? read_slot(token, true) : std::nullopt;
        if (!younger || too_old(*younger))
        {
          return Event{EventKind::StaleValue, subject, interface, writer_index, now};
        }
        return std::nullopt;
      }
    }
    if (!record || depth == MAX_CHAIN_DEPTH)
    {
      return Event{EventKind::UnknownValue, subject, interface, NO_INDEX, now};
    }
    if (expected_interface != NO_INDEX && record->interface != expected_interface)
    {
      return Event{EventKind::MisroutedValue, subject, interface, record->controller, now};
    }
    const auto writer = controller(record->controller);
    const bool stopped_recently =
      grace_cycles > 0 &&
      now <= controllers_.at(record->controller).left_active_cycle.load() + grace_cycles;
    const bool running = writer.state == Lifecycle::Active && writer.epoch == record->epoch;
    if (!running && !stopped_recently)
    {
      return Event{EventKind::InactiveWriter, subject, interface, record->controller, now};
    }
    if (too_old(*record))
    {
      return Event{EventKind::StaleValue, subject, interface, record->controller, now};
    }
    if (record->parent == 0)
    {
      return std::nullopt;
    }
    token = record->parent;
    // A parent is the reference the writer read, its interface is checked by the writer itself
    expected_interface = NO_INDEX;
  }
}

void Trace::begin_hang(Target target)
{
  hangs_running_.fetch_add(1);
  if (target.kind == TargetKind::Controller)
  {
    controllers_.at(target.index).hanging.fetch_add(1);
  }
  else
  {
    hardware_.at(target.index).hanging.fetch_add(1);
  }
}

void Trace::end_hang(Target target)
{
  if (target.kind == TargetKind::Controller)
  {
    controllers_.at(target.index).hanging.fetch_sub(1);
  }
  else
  {
    hardware_.at(target.index).hanging.fetch_sub(1);
  }
  hangs_running_.fetch_sub(1);
}

int Trace::hangs_running() const { return hangs_running_.load(); }

void Trace::begin_call() { calls_running_.fetch_add(1); }

void Trace::end_call() { calls_running_.fetch_sub(1); }

int Trace::calls_running() const { return calls_running_.load(); }

bool Trace::controller_hanging(uint16_t controller) const
{
  return controllers_.at(controller).hanging.load() > 0;
}

bool Trace::hardware_hanging(uint16_t hardware) const
{
  return hardware_.at(hardware).hanging.load() > 0;
}

void Trace::report(const Event & event)
{
  std::lock_guard<std::mutex> lock(events_mutex_);
  // A full buffer still keeps the first event of every kind, so a flood never hides a gate
  const bool first_of_kind = std::none_of(
    events_.begin(), events_.end(), [&](const Event & e) { return e.kind == event.kind; });
  if (events_.size() >= EVENT_CAPACITY && !first_of_kind)
  {
    events_overflowed_ = true;
    return;
  }
  events_.push_back(event);
}

std::vector<Event> Trace::take_events()
{
  std::lock_guard<std::mutex> lock(events_mutex_);
  std::vector<Event> taken;
  taken.reserve(EVENT_RESERVE);
  std::swap(taken, events_);
  if (events_overflowed_)
  {
    taken.push_back(
      {EventKind::EventsLost, {TargetKind::Hardware, 0}, NO_INDEX, NO_INDEX, cycle()});
    events_overflowed_ = false;
  }
  return taken;
}

Trace & trace()
{
  static Trace instance;
  return instance;
}

}  // namespace cm_test_suite
