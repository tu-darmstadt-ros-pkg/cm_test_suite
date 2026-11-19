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

#include "operation.hpp"

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <future>

#include "cm_test_suite/trace.hpp"

namespace cm_test_suite
{
namespace
{
constexpr std::chrono::seconds HUNG_GRACE{30};

bool ready(const std::future<bool> & result)
{
  return result.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}
}  // namespace

OperationResult run_operation(
  ControlLoop & loop, const std::function<bool()> & operation, std::vector<TimedAction> actions,
  ControlLoop::Clock::duration hang_limit)
{
  std::stable_sort(
    actions.begin(), actions.end(),
    [](const TimedAction & a, const TimedAction & b) { return a.after_cycles < b.after_cycles; });
  auto next = actions.begin();
  for (; next != actions.end() && next->after_cycles == 0; ++next)
  {
    next->action();
  }
  OperationResult result;
  const auto start = ControlLoop::Clock::now();
  const uint64_t start_cycle = trace().cycle();
  // Measured where the operation runs, so a harness stall on this thread does not count
  std::atomic<ControlLoop::Clock::rep> finished{0};
  auto future = std::async(
    std::launch::async,
    [&operation, &finished]
    {
      struct Stamp
      {
        std::atomic<ControlLoop::Clock::rep> & at;
        ~Stamp() { at = ControlLoop::Clock::now().time_since_epoch().count(); }
      } stamp{finished};
      return operation();
    });
  while (!ready(future))
  {
    for (; next != actions.end() && trace().cycle() - start_cycle >= next->after_cycles; ++next)
    {
      next->action();
    }
    if (ControlLoop::Clock::now() - start > hang_limit && !result.hung)
    {
      result.hung = true;
      result.error = "the operation did not return";
    }
    if (ControlLoop::Clock::now() - start > hang_limit + HUNG_GRACE)
    {
      std::fprintf(stderr, "An operation hung for good, aborting\n");
      std::abort();
    }
    loop.tick();
  }
  result.duration =
    ControlLoop::Clock::time_point(ControlLoop::Clock::duration(finished.load())) - start;
  try
  {
    const bool accepted = future.get();
    if (!result.hung)
    {
      result.accepted = accepted;
    }
  }
  catch (const std::exception & e)
  {
    result.error = std::string("the operation threw: ") + e.what();
  }
  return result;
}

}  // namespace cm_test_suite
