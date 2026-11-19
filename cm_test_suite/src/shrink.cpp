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

#include "cm_test_suite/shrink.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cctype>
#include <utility>
#include <vector>

namespace cm_test_suite
{
namespace
{
using Runs = std::function<bool(const Script &)>;

/// Removes chunks of steps while the script keeps failing (delta debugging).
bool drop_steps(Script & current, const Runs & fails, const std::function<bool()> & may_run)
{
  bool any = false;
  size_t chunks = 2;
  while (current.size() > 1 && may_run())
  {
    const size_t chunk = (current.size() + chunks - 1) / chunks;
    bool reduced = false;
    for (size_t start = 0; start < current.size() && may_run(); start += chunk)
    {
      Script candidate(current.begin(), current.begin() + static_cast<std::ptrdiff_t>(start));
      const size_t end = std::min(start + chunk, current.size());
      candidate.insert(
        candidate.end(), current.begin() + static_cast<std::ptrdiff_t>(end), current.end());
      if (fails(candidate))
      {
        current = std::move(candidate);
        chunks = std::max<size_t>(chunks - 1, 2);
        reduced = any = true;
        break;
      }
    }
    if (!reduced)
    {
      if (chunk == 1)
      {
        break;
      }
      chunks = std::min(chunks * 2, current.size());
    }
  }
  return any;
}

template <typename T, typename Entry>
void without_each(const T & step, std::vector<Entry> T::* list, std::vector<Step> & variants)
{
  for (size_t i = 0; i < (step.*list).size(); ++i)
  {
    T variant = step;
    (variant.*list).erase((variant.*list).begin() + static_cast<std::ptrdiff_t>(i));
    variants.push_back(std::move(variant));
  }
}

/// Resets an optional fault or a flag that is set.
template <typename T, typename Member>
void without(const T & step, Member T::* member, std::vector<Step> & variants)
{
  if (step.*member)
  {
    variants.push_back(step);
    std::get<T>(variants.back()).*member = Member{};
  }
}

/// Waits where the step did not.
template <typename T>
void waiting(const T & step, std::vector<Step> & variants)
{
  if (!step.wait)
  {
    variants.push_back(step);
    std::get<T>(variants.back()).wait = true;
  }
}

/// The step with one fault, stall, hold, power cut, nowait or list entry less.
std::vector<Step> simpler(const Step & step)
{
  std::vector<Step> variants;
  if (const auto * s = std::get_if<SwitchStep>(&step))
  {
    without_each(*s, &SwitchStep::activate, variants);
    without_each(*s, &SwitchStep::deactivate, variants);
    without_each(*s, &SwitchStep::faults, variants);
    without(*s, &SwitchStep::stall, variants);
    without(*s, &SwitchStep::hold, variants);
    without(*s, &SwitchStep::power_cut, variants);
  }
  else if (const auto * f = std::get_if<FaultStep>(&step))
  {
    without_each(*f, &FaultStep::handling, variants);
    waiting(*f, variants);
  }
  else if (const auto * l = std::get_if<LifecycleStep>(&step))
  {
    without(*l, &LifecycleStep::fault, variants);
    waiting(*l, variants);
  }
  else if (const auto * r = std::get_if<RecoverStep>(&step))
  {
    without(*r, &RecoverStep::fault, variants);
  }
  else if (const auto * d = std::get_if<ShutdownStep>(&step))
  {
    without(*d, &ShutdownStep::fault, variants);
    without(*d, &ShutdownStep::stall, variants);
  }
  return variants;
}

/// Replaces steps by simpler variants while the script keeps failing.
bool simplify_steps(Script & current, const Runs & fails, const std::function<bool()> & may_run)
{
  bool any = false;
  for (size_t i = 0; i < current.size() && may_run(); ++i)
  {
    bool reduced = true;
    while (reduced && may_run())
    {
      reduced = false;
      for (auto & variant : simpler(current[i]))
      {
        Script candidate = current;
        candidate[i] = std::move(variant);
        if (may_run() && fails(candidate))
        {
          current = std::move(candidate);
          reduced = any = true;
          break;
        }
      }
    }
  }
  return any;
}
}  // namespace

Script shrink(
  const Script & script, const std::function<bool(const Script &)> & fails, size_t max_runs,
  std::chrono::steady_clock::duration budget)
{
  const auto deadline = std::chrono::steady_clock::now() + budget;
  size_t runs = 0;
  const std::function<bool()> may_run = [&]
  { return runs < max_runs && std::chrono::steady_clock::now() < deadline; };
  // A failure that shows only now and then would otherwise stop the shrink early
  const Runs fails_once_in_two = [&](const Script & candidate)
  {
    for (int attempt = 0; attempt < 2 && may_run(); ++attempt)
    {
      ++runs;
      if (fails(candidate))
      {
        return true;
      }
    }
    return false;
  };
  Script current = script;
  drop_steps(current, fails_once_in_two, may_run);
  // Each pass may make room for the other or for itself
  for (bool shrunk = true; shrunk && may_run();)
  {
    shrunk = simplify_steps(current, fails_once_in_two, may_run);
    shrunk = drop_steps(current, fails_once_in_two, may_run) || shrunk;
  }
  return current;
}

std::string failure_signature(const std::string & failure)
{
  // A failure reads "[tier] [category] <phase>: <message>", a step phase being "step <n> '<line>'"
  // followed by " then '<line>'" for each call of the power recovery
  const size_t phase = failure.find("] ", failure.find("] ") + 2) + 2;
  size_t message = failure.find(": ", phase);
  if (failure.compare(phase, 5, "step ") == 0)
  {
    message = failure.find('\'', phase);
    while (true)
    {
      message = failure.find('\'', message + 1) + 1;
      if (failure.compare(message, 7, " then '") != 0)
      {
        break;
      }
      message += 6;
    }
  }
  std::string signature = failure.substr(0, failure.find_first_of(" :", phase));
  // Name digits like a1 stay, numbers go and so do name lists, which a dropped bystander changes
  bool in_name = false;
  bool in_list = false;
  for (const unsigned char c : failure.substr(std::min(message, failure.size())))
  {
    if (in_list && c != ']')
    {
      continue;
    }
    in_list = c == '[';
    if (!std::isdigit(c))
    {
      in_name = std::isalpha(c) || c == '_' || c == '/';
    }
    if (in_name || !std::isdigit(c))
    {
      signature += static_cast<char>(c);
    }
  }
  return signature;
}

void report_and_shrink(
  const std::string & failure, const Script & script,
  const std::function<bool(const Script &)> & fails, size_t max_runs,
  std::chrono::steady_clock::duration budget)
{
  ADD_FAILURE() << failure << "\nShrinking the script for at most "
                << std::chrono::duration_cast<std::chrono::seconds>(budget).count() << " s";
  const Script minimal = shrink(script, fails, max_runs, budget);
  ADD_FAILURE() << "Shrunk script, save it and replay it with CM_FUZZ_SCRIPT=<file>:\n"
                << to_text(minimal);
}

}  // namespace cm_test_suite
