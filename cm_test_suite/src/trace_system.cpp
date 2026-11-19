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
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "cm_test_suite/rig.hpp"
#include "hardware_interface/system_interface.hpp"
#include "plugin_support.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace cm_test_suite
{
using hardware_interface::return_type;

/// Rig hardware that behaves like a Hector drive without watchdog.
///
/// A command interface applies commands only between a start and a stop of its mode, a stop
/// holds it. Every applied command must lie within the joint limits and be a live token of a
/// running chain or a probe clamped onto a limit. A joint accepts one mode at a time, prepare
/// rejects anything else.
class TraceSystem : public hardware_interface::SystemInterface
{
public:
  ~TraceSystem() override
  {
    if (index_)
    {
      trace().set_hardware_state(*index_, Lifecycle::Unloaded);
    }
  }

  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }
    index_ = rig::hardware_index(get_hardware_info().name);
    if (!index_)
    {
      RCLCPP_ERROR(get_logger(), "Hardware %s is not part of the rig", get_name().c_str());
      return CallbackReturn::ERROR;
    }
    definition_ = &rig::hardware()[*index_];
    target_ = {TargetKind::Hardware, *index_};
    trace().set_hardware_state(*index_, Lifecycle::Unconfigured);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    auto busy = guard();
    const PendingCall pending;
    if (auto forced = take_lifecycle_fault(target_, Hook::Configure, busy))
    {
      return *forced;
    }
    bind_commands();
    settle(Lifecycle::Inactive);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    auto busy = guard();
    const PendingCall pending;
    if (!powered())
    {
      return CallbackReturn::ERROR;
    }
    if (auto forced = take_lifecycle_fault(target_, Hook::Activate, busy))
    {
      return *forced;
    }
    settle(Lifecycle::Active);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    auto busy = guard();
    const PendingCall pending;
    if (auto forced = take_lifecycle_fault(target_, Hook::Deactivate, busy))
    {
      return *forced;
    }
    settle(Lifecycle::Inactive);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    auto busy = guard();
    const PendingCall pending;
    if (auto forced = take_lifecycle_fault(target_, Hook::Cleanup, busy))
    {
      return *forced;
    }
    settle(Lifecycle::Unconfigured);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State &) override
  {
    auto busy = guard();
    const PendingCall pending;
    if (auto forced = take_lifecycle_fault(target_, Hook::Error, busy))
    {
      settle(Lifecycle::Finalized);
      return *forced;
    }
    settle(Lifecycle::Unconfigured);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    auto busy = guard();
    const PendingCall pending;
    settle(Lifecycle::Finalized);
    return CallbackReturn::SUCCESS;
  }

  return_type prepare_command_mode_switch(
    const std::vector<std::string> & start, const std::vector<std::string> & stop) override
  {
    auto busy = guard();
    const PendingCall pending;
    if (auto result = take_switch_fault(Hook::Prepare))
    {
      return *result;
    }
    return modes_valid(live_after(start, stop)) ? return_type::OK : return_type::ERROR;
  }

  return_type perform_command_mode_switch(
    const std::vector<std::string> & start, const std::vector<std::string> & stop) override
  {
    auto busy = guard();
    const PendingCall pending;
    if (auto result = take_switch_fault(Hook::Perform))
    {
      return *result;
    }
    const auto live = live_after(start, stop);
    if (!modes_valid(live))
    {
      report(EventKind::ModeConflict);
    }
    for (size_t i = 0; i < commands_.size(); ++i)
    {
      set_live(commands_[i], live[i]);
    }
    return return_type::OK;
  }

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    auto busy = guard();
    if (!definition_->is_async && trace().wait_while_held())
    {
      trace().mark_held(*index_);
    }
    const return_type result = io(Hook::Read, busy).value_or(return_type::OK);
    if (result == return_type::OK)
    {
      trace().count_read(*index_);
    }
    if (definition_->is_async)
    {
      trace().pace_async_io(false);
    }
    return result;
  }

  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    auto busy = guard();
    if (const auto result = io(Hook::Write, busy))
    {
      return *result;
    }
    // Waiting runnable since the last write, mostly for the wakeup by the trigger, does not age
    // Only the fuzz runs cover this per write baseline
    const auto applied_at = std::chrono::steady_clock::now();
    const auto preempted = definition_->is_async && preempted_after_write_
                             ? preempted_time() - *preempted_after_write_
                             : std::chrono::nanoseconds{};
    // The records of the tokens this write applied so far, preallocated by bind_commands
    auto & applied = applied_records_;
    applied.clear();
    for (const auto & command : commands_)
    {
      double value = 0.0;
      // A handle busy with a controller write is retried next cycle, like a real driver would
      if (!command.live || !get_command(command.handle, value, false) || std::isnan(value))
      {
        continue;
      }
      trace().set_interface_applied(command.interface);
      if (value < command.lower || value > command.upper)
      {
        trace().report(
          {EventKind::LimitExceeded, target_, command.interface, NO_INDEX, trace().cycle()});
        continue;
      }
      // A probe clamped into the limits carries no token, also one a limiter cut below a limit to
      // keep the position in range within the cycle
      if (std::fabs(value) >= rig::POSITION_LIMIT)
      {
        continue;
      }
      const auto applier = definition_->is_async ? Applier::AsyncThread : Applier::ControlLoop;
      const uint64_t token = decode_token(value);
      if (
        auto event =
          trace().audit(token, command.interface, target_, applier, preempted, applied_at))
      {
        trace().report(*event);
      }
      report_torn(command.interface, trace().lookup(token), applied);
    }
    trace().count_write(*index_);
    if (definition_->is_async)
    {
      trace().pace_async_io(true);
      preempted_after_write_ = preempted_time();
    }
    return return_type::OK;
  }

private:
  struct Command
  {
    hardware_interface::CommandInterface::SharedPtr handle;
    uint16_t interface;
    size_t joint;
    std::string name;
    /// The band its joint limits allow, unbounded without limits.
    double lower = -std::numeric_limits<double>::infinity();
    double upper = std::numeric_limits<double>::infinity();
    /// Written by the switch calls and read by write, which must never overlap.
    std::atomic<bool> live{false};
  };

  CallGuard guard() { return CallGuard(busy_, EventKind::ConcurrentHardwareCall, target_); }

  /// A controller writes all its commands in one update, so one write must apply them from one.
  void report_torn(
    uint16_t interface, const std::optional<TokenRecord> & record,
    std::vector<TokenRecord> & applied) const
  {
    if (!record)
    {
      return;
    }
    for (const auto & other : applied)
    {
      if (
        other.controller == record->controller && other.epoch == record->epoch &&
        other.cycle != record->cycle)
      {
        trace().report(
          {EventKind::TornCommand, target_, interface, record->controller, trace().cycle()});
        break;
      }
    }
    applied.push_back(*record);
  }

  void bind_commands()
  {
    if (!commands_.empty())
    {
      return;
    }
    const auto & joints = definition_->joints;
    commands_ = std::vector<Command>(joints.size() * definition_->command_modes.size());
    applied_records_.reserve(commands_.size());
    size_t i = 0;
    for (size_t joint = 0; joint < joints.size(); ++joint)
    {
      for (const auto & mode : definition_->command_modes)
      {
        auto & command = commands_[i++];
        command.name = joints[joint] + "/" + mode;
        command.handle = get_command_interface_handle(command.name);
        command.interface = *rig::interface_id(command.name);
        command.joint = joint;
        const auto limits = get_hardware_info().limits.find(joints[joint]);
        if (limits == get_hardware_info().limits.end())
        {
          continue;
        }
        if (mode == "position" && limits->second.has_position_limits)
        {
          command.lower = limits->second.min_position;
          command.upper = limits->second.max_position;
        }
        if (mode == "velocity" && limits->second.has_velocity_limits)
        {
          command.lower = -limits->second.max_velocity;
          command.upper = limits->second.max_velocity;
        }
      }
    }
  }

  /// Enters a lifecycle state. Every transition holds all commands and clears a fault.
  void settle(Lifecycle state)
  {
    for (auto & command : commands_)
    {
      set_live(command, false);
    }
    trace().set_hardware_faulted(*index_, false);
    trace().set_hardware_state(*index_, state);
  }

  static void set_live(Command & command, bool live)
  {
    command.live = live;
    trace().set_interface_live(command.interface, live);
  }

  bool powered() const { return trace().power() || !definition_->faults_on_power_loss; }

  /// The result the call returns right away, if any. A hung write sends nothing, like a transfer
  /// that timed out.
  std::optional<return_type> io(Hook hook, CallGuard & call)
  {
    const auto truth = trace().hardware(*index_);
    // State interfaces of inactive hardware are available, so it is read but never written
    const bool allowed = truth.state == Lifecycle::Active ||
                         (hook == Hook::Read && truth.state == Lifecycle::Inactive);
    if (!allowed)
    {
      report(EventKind::IoWhileInactive);
    }
    if (truth.faulted)
    {
      report(EventKind::IoAfterFault);
    }
    if (!powered())
    {
      return ++failed_transfers_ >= rig::POWER_LOSS_ERROR_AFTER
               ? std::optional(fail(return_type::ERROR))
               : std::nullopt;
    }
    failed_transfers_ = 0;
    switch (faults().take(target_, hook))
    {
      case Fault::Error:
        return fail(return_type::ERROR);
      case Fault::Deactivate:
        return fail(return_type::DEACTIVATE);
      case Fault::Throw:
        trace().set_hardware_faulted(*index_, true);
        throw InjectedFault(hook);
      case Fault::Hang:
        call.begin_hang(target_);
        std::this_thread::sleep_for(HANG);
        return hook == Hook::Write ? std::optional(return_type::OK) : std::nullopt;
      default:
        return std::nullopt;
    }
  }

  return_type fail(return_type result)
  {
    trace().set_hardware_faulted(*index_, true);
    return result;
  }

  std::optional<return_type> take_switch_fault(Hook hook)
  {
    switch (faults().take(target_, hook))
    {
      case Fault::Error:
        return return_type::ERROR;
      case Fault::Throw:
        throw InjectedFault(hook);
      case Fault::Delay:
        std::this_thread::sleep_for(SLOW_SWITCH);
        return std::nullopt;
      default:
        return std::nullopt;
    }
  }

  std::vector<bool> live_after(
    const std::vector<std::string> & start, const std::vector<std::string> & stop) const
  {
    const auto contains = [](const std::vector<std::string> & names, const std::string & name)
    { return std::find(names.begin(), names.end(), name) != names.end(); };
    std::vector<bool> live(commands_.size());
    for (size_t i = 0; i < commands_.size(); ++i)
    {
      const auto & name = commands_[i].name;
      live[i] = contains(start, name) || (commands_[i].live && !contains(stop, name));
    }
    return live;
  }

  bool modes_valid(const std::vector<bool> & live) const
  {
    std::vector<int> modes_per_joint(definition_->joints.size(), 0);
    for (size_t i = 0; i < commands_.size(); ++i)
    {
      modes_per_joint[commands_[i].joint] += live[i] ? 1 : 0;
    }
    return std::all_of(
      modes_per_joint.begin(), modes_per_joint.end(), [](int modes) { return modes <= 1; });
  }

  void report(EventKind kind) const
  {
    trace().report({kind, target_, NO_INDEX, NO_INDEX, trace().cycle()});
  }

  std::optional<uint16_t> index_;
  const rig::Hardware * definition_ = nullptr;
  Target target_{TargetKind::Hardware, 0};
  std::vector<Command> commands_;
  std::atomic<bool> busy_{false};
  /// Run queue wait of the async owner thread when its last write ended.
  /// The difference is valid only because the same owner thread runs every read and write.
  std::optional<std::chrono::nanoseconds> preempted_after_write_;
  std::vector<TokenRecord> applied_records_;
  /// Read and write may run on different threads, one at a time.
  std::atomic<int> failed_transfers_{0};
};

}  // namespace cm_test_suite

PLUGINLIB_EXPORT_CLASS(cm_test_suite::TraceSystem, hardware_interface::SystemInterface)
