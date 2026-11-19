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

#ifndef PLUGIN_SUPPORT_HPP_
#define PLUGIN_SUPPORT_HPP_

#include <array>
#include <atomic>
#include <exception>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include "cm_test_suite/faults.hpp"
#include "cm_test_suite/trace.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace cm_test_suite
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/// Thrown by a rig plugin on an injected fault. It allocates nothing, so a throw inside the control
/// cycle is never counted as an allocation of the controller manager.
class InjectedFault : public std::exception
{
public:
  explicit InjectedFault(Hook hook) : hook_(hook) {}

  const char * what() const noexcept override
  {
    static constexpr std::array<const char *, static_cast<size_t>(Hook::Count)> MESSAGES = {
      "Injected fault in read",         "Injected fault in write",
      "Injected fault in prepare",      "Injected fault in perform",
      "Injected fault in configure",    "Injected fault in activate",
      "Injected fault in deactivate",   "Injected fault in cleanup",
      "Injected fault in error",        "Injected fault in update",
      "Injected fault in chained_mode", "Injected fault in shutdown"};
    return MESSAGES[static_cast<size_t>(hook_)];
  }

private:
  Hook hook_;
};

/// Counts a running injected hang or late callback, so the harness can wait it out before it checks
/// anything.
class HangScope
{
public:
  explicit HangScope(Target target) : target_(target) { trace().begin_hang(target_); }
  ~HangScope() { trace().end_hang(target_); }

  HangScope(const HangScope &) = delete;
  HangScope & operator=(const HangScope &) = delete;

private:
  Target target_;
};

/// Counts a lifecycle or mode switch callback while it runs.
class PendingCall
{
public:
  PendingCall() { trace().begin_call(); }
  ~PendingCall() { trace().end_call(); }

  PendingCall(const PendingCall &) = delete;
  PendingCall & operator=(const PendingCall &) = delete;
};

/// Reports a call that overlaps another call into the same plugin. An injected hang in the call
/// counts until the call returns, so the harness never sees the plugin before its own state change.
class CallGuard
{
public:
  CallGuard(std::atomic<bool> & busy, EventKind kind, Target subject)
  : busy_(busy), entered_(!busy.exchange(true))
  {
    if (!entered_)
    {
      trace().report({kind, subject, NO_INDEX, NO_INDEX, trace().cycle()});
    }
  }

  ~CallGuard()
  {
    if (entered_)
    {
      busy_ = false;
    }
  }

  CallGuard(const CallGuard &) = delete;
  CallGuard & operator=(const CallGuard &) = delete;

  void begin_hang(Target target) { hang_.emplace(target); }

private:
  std::atomic<bool> & busy_;
  const bool entered_;
  std::optional<HangScope> hang_;
};

/// Applies a fault taken for a lifecycle callback. Returns the result it forces, if any.
///
/// A hang outlives every timeout and the harness waits it out before it checks anything. A
/// hanging activation gives up with FAILURE, leaving no active plugin behind that the controller
/// manager already gave up on.
inline std::optional<CallbackReturn> apply_lifecycle_fault(
  Fault fault, Target target, Hook hook, CallGuard & call)
{
  switch (fault)
  {
    case Fault::Failure:
      return CallbackReturn::FAILURE;
    case Fault::Error:
      return CallbackReturn::ERROR;
    case Fault::Throw:
      throw InjectedFault(hook);
    case Fault::Delay:
      std::this_thread::sleep_for(DELAY);
      return std::nullopt;
    case Fault::Late:
      // The controller manager may give up on it at its deadline, like on a hang (rulebook 2.2.5)
      call.begin_hang(target);
      std::this_thread::sleep_for(LATE);
      return std::nullopt;
    case Fault::Hang:
      call.begin_hang(target);
      std::this_thread::sleep_for(HANG);
      return hook == Hook::Activate ? std::optional(CallbackReturn::FAILURE) : std::nullopt;
    case Fault::None:
    case Fault::Deactivate:
    case Fault::Quiet:
      // FaultBoard::arm rejects Deactivate for lifecycle hooks, the controller handles Quiet
      return std::nullopt;
  }
  return std::nullopt;
}

/// Takes and applies the fault armed for a lifecycle callback.
inline std::optional<CallbackReturn> take_lifecycle_fault(
  Target target, Hook hook, CallGuard & call)
{
  return apply_lifecycle_fault(faults().take(target, hook), target, hook, call);
}

}  // namespace cm_test_suite

#endif  // PLUGIN_SUPPORT_HPP_
