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

#include "control_loop.hpp"

#include <stdexcept>
#include <utility>

#include "cm_test_suite/rt_allocations.hpp"
#include "cm_test_suite/trace.hpp"

namespace cm_test_suite
{
namespace
{
constexpr std::chrono::milliseconds PERIOD{1};
constexpr std::chrono::seconds LOOP_STOPPED_TIMEOUT{10};
}  // namespace

ControlLoop::ControlLoop(CmUnderTest & cm, Mode mode) : cm_(cm), mode_(mode)
{
  // Throws here with its message rather than terminating inside the loop thread
  preempted_time();
  if (mode_ == Mode::Threaded)
  {
    thread_ = std::thread(
      [this]
      {
        while (running_)
        {
          if (!paused_)
          {
            cycle();
          }
          std::this_thread::sleep_for(PERIOD);
        }
      });
  }
}

ControlLoop::~ControlLoop() { stop(); }

void ControlLoop::tick()
{
  if (mode_ == Mode::Stepped)
  {
    cycle();
  }
  else
  {
    std::this_thread::sleep_for(PERIOD);
  }
}

void ControlLoop::run_cycles(size_t cycles)
{
  if (mode_ == Mode::Stepped)
  {
    for (size_t i = 0; i < cycles; ++i)
    {
      cycle();
    }
    return;
  }
  const uint64_t target = trace().cycle() + cycles;
  const auto deadline = Clock::now() + LOOP_STOPPED_TIMEOUT;
  while (trace().cycle() < target)
  {
    if (Clock::now() > deadline)
    {
      throw std::runtime_error("The control loop stopped cycling");
    }
    std::this_thread::sleep_for(PERIOD);
  }
}

void ControlLoop::stall(Clock::duration duration, const std::function<bool()> & until)
{
  paused_ = true;
  const auto end = Clock::now() + duration;
  while (Clock::now() < end && !until())
  {
    std::this_thread::sleep_for(PERIOD);
  }
  paused_ = false;
}

void ControlLoop::stop()
{
  running_ = false;
  if (thread_.joinable())
  {
    thread_.join();
  }
}

ControlLoop::Clock::duration ControlLoop::take_longest_cycle()
{
  return Clock::duration(longest_cycle_.exchange(0));
}

std::vector<std::string> ControlLoop::take_errors()
{
  std::lock_guard<std::mutex> lock(errors_mutex_);
  return std::exchange(errors_, {});
}

void ControlLoop::cycle()
{
  if (!running_)
  {
    return;
  }
  trace().begin_cycle();
  const auto start = Clock::now();
  const auto preempted_before = preempted_time();
  try
  {
    RtScope rt;
    cm_.cycle();
  }
  catch (const std::exception & e)
  {
    std::lock_guard<std::mutex> lock(errors_mutex_);
    errors_.push_back(e.what());
  }
  // Other threads preempting the cycle on a shared core is not blocking by the controller manager
  const auto preempted = preempted_time() - preempted_before;
  trace().end_cycle(preempted);
  const auto duration = (Clock::now() - start - preempted).count();
  for (auto longest = longest_cycle_.load(); duration > longest;)
  {
    if (longest_cycle_.compare_exchange_weak(longest, duration))
    {
      break;
    }
  }
}

}  // namespace cm_test_suite
