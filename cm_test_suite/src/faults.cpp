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

#include "cm_test_suite/faults.hpp"

#include <stdexcept>
#include <string>

namespace cm_test_suite
{
namespace
{
constexpr std::array HOOK_NAMES = {"read",      "write",    "prepare",      "perform",
                                   "configure", "activate", "deactivate",   "cleanup",
                                   "error",     "update",   "chained_mode", "shutdown"};
constexpr std::array FAULT_NAMES = {"none",  "error", "failure", "deactivate", "throw",
                                    "delay", "hang",  "quiet",   "late"};

static_assert(HOOK_NAMES.size() == static_cast<size_t>(Hook::Count));

template <typename Enum, typename Names>
std::optional<Enum> parse(const Names & names, std::string_view text)
{
  for (size_t i = 0; i < names.size(); ++i)
  {
    if (text == names[i])
    {
      return static_cast<Enum>(i);
    }
  }
  return std::nullopt;
}
}  // namespace

const char * to_string(Hook hook) { return HOOK_NAMES.at(static_cast<size_t>(hook)); }

const char * to_string(Fault fault) { return FAULT_NAMES.at(static_cast<size_t>(fault)); }

std::optional<Hook> parse_hook(std::string_view text) { return parse<Hook>(HOOK_NAMES, text); }

std::optional<Fault> parse_fault(std::string_view text) { return parse<Fault>(FAULT_NAMES, text); }

bool applicable(TargetKind kind, Hook hook, Fault fault)
{
  const bool is_hardware = kind == TargetKind::Hardware;
  switch (hook)
  {
    case Hook::Read:
    case Hook::Write:
      return is_hardware && (fault == Fault::Error || fault == Fault::Deactivate ||
                             fault == Fault::Throw || fault == Fault::Hang);
    case Hook::Prepare:
    case Hook::Perform:
      return is_hardware &&
             (fault == Fault::Error || fault == Fault::Throw || fault == Fault::Delay);
    case Hook::Update:
      return !is_hardware && (fault == Fault::Error || fault == Fault::Throw);
    case Hook::ChainedMode:
      return !is_hardware && fault == Fault::Failure;
    case Hook::Deactivate:
      // Only a controller may refuse to stop, it then counts as stopped
      return (fault != Fault::Failure || !is_hardware) &&
             (fault == Fault::Failure || fault == Fault::Error || fault == Fault::Throw ||
              fault == Fault::Delay || fault == Fault::Hang || fault == Fault::Late);
    case Hook::Shutdown:
      return !is_hardware &&
             (fault == Fault::Failure || fault == Fault::Error || fault == Fault::Throw ||
              fault == Fault::Delay || fault == Fault::Hang);
    case Hook::Activate:
      if (fault == Fault::Quiet)
      {
        return !is_hardware;
      }
      [[fallthrough]];
    case Hook::Error:
      if (fault == Fault::Late)
      {
        return true;
      }
      [[fallthrough]];
    case Hook::Configure:
    case Hook::Cleanup:
      return fault == Fault::Failure || fault == Fault::Error || fault == Fault::Throw ||
             fault == Fault::Delay || fault == Fault::Hang;
    case Hook::Count:
      return false;
  }
  return false;
}

void FaultBoard::arm(Target target, Hook hook, Fault fault)
{
  if (!applicable(target.kind, hook, fault))
  {
    throw std::invalid_argument(
      std::string("Fault ") + to_string(fault) + " is not applicable at " + to_string(hook));
  }
  armed_[slot(target, hook)] = fault;
}

Fault FaultBoard::take(Target target, Hook hook)
{
  const size_t index = slot(target, hook);
  const Fault fault = armed_[index].exchange(Fault::None);
  if (
    fault == Fault::Error || fault == Fault::Failure || fault == Fault::Deactivate ||
    fault == Fault::Throw || fault == Fault::Hang || fault == Fault::Late)
  {
    fired_[index] = true;
  }
  return fault;
}

bool FaultBoard::pending(Target target, Hook hook) const
{
  return armed_[slot(target, hook)] != Fault::None;
}

void FaultBoard::clear()
{
  for (size_t i = 0; i < SLOTS; ++i)
  {
    armed_[i] = Fault::None;
    fired_[i] = false;
  }
}

std::vector<Target> FaultBoard::take_fired()
{
  std::vector<Target> fired;
  const size_t hooks = static_cast<size_t>(Hook::Count);
  for (size_t i = 0; i < SLOTS; ++i)
  {
    if (fired_[i].exchange(false))
    {
      const size_t target = i / hooks;
      fired.push_back(
        {target < MAX_TARGETS ? TargetKind::Controller : TargetKind::Hardware,
         static_cast<uint16_t>(target % MAX_TARGETS)});
    }
  }
  return fired;
}

size_t FaultBoard::slot(Target target, Hook hook) const
{
  if (target.index >= MAX_TARGETS || hook == Hook::Count)
  {
    throw std::out_of_range("Fault target or hook out of range");
  }
  const size_t kind = target.kind == TargetKind::Controller ? 0 : 1;
  return (kind * MAX_TARGETS + target.index) * static_cast<size_t>(Hook::Count) +
         static_cast<size_t>(hook);
}

FaultBoard & faults()
{
  static FaultBoard instance;
  return instance;
}

}  // namespace cm_test_suite
