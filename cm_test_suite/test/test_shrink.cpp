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

#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <string>
#include <thread>

#include "cm_test_suite/shrink.hpp"

namespace cm_test_suite
{
namespace
{
using std::chrono::milliseconds;
using std::chrono::seconds;

bool contains(const Script & script, const std::string & controller)
{
  return std::any_of(
    script.begin(), script.end(),
    [&](const Step & step)
    {
      const auto * s = std::get_if<LifecycleStep>(&step);
      return s && s->controller == controller;
    });
}
}  // namespace

TEST(Shrink, keeps_only_the_steps_needed_to_fail)
{
  Script script;
  for (const auto & c : rig::controllers())
  {
    script.push_back(LifecycleStep{LifecycleAction::Load, c.name, std::nullopt, std::nullopt});
  }
  const auto fails = [](const Script & s) { return contains(s, "wrist") && contains(s, "nav"); };
  const Script minimal = shrink(script, fails, 200, seconds(60));
  EXPECT_EQ(2u, minimal.size());
  EXPECT_TRUE(fails(minimal));
}

TEST(Shrink, stops_at_the_run_budget)
{
  Script script(50, LifecycleStep{LifecycleAction::Load, "nav", std::nullopt, std::nullopt});
  size_t runs = 0;
  shrink(
    script,
    [&](const Script &)
    {
      ++runs;
      return false;
    },
    7, seconds(60));
  EXPECT_EQ(7u, runs);
}

TEST(Shrink, stops_at_the_time_budget_with_the_smallest_script_so_far)
{
  Script script(1024, LifecycleStep{LifecycleAction::Load, "nav", std::nullopt, std::nullopt});
  script.front() = LifecycleStep{LifecycleAction::Load, "wrist", std::nullopt, std::nullopt};
  size_t runs = 0;
  const Script minimal = shrink(
    script,
    [&](const Script & s)
    {
      ++runs;
      std::this_thread::sleep_for(milliseconds(50));
      return contains(s, "wrist");
    },
    1000, milliseconds(300));
  // Shrinking to one step takes 20 runs, so the budget stops it on the way
  EXPECT_LT(runs, 15u);
  EXPECT_LT(minimal.size(), script.size());
  EXPECT_GT(minimal.size(), 1u);
  EXPECT_TRUE(contains(minimal, "wrist"));
}

TEST(Shrink, counts_a_candidate_failing_in_one_of_two_runs)
{
  Script script;
  for (const auto & c : rig::controllers())
  {
    script.push_back(LifecycleStep{LifecycleAction::Load, c.name, std::nullopt, std::nullopt});
  }
  size_t runs = 0;
  // Fails only in every second run
  const auto flaky = [&](const Script & s) { return contains(s, "wrist") && ++runs % 2 == 0; };
  EXPECT_EQ(1u, shrink(script, flaky, 200, seconds(60)).size());
}

TEST(Shrink, drops_faults_stalls_holds_power_cuts_nowait_and_single_entries_of_the_failing_steps)
{
  const Script script = parse_script(
    "load nav\n"
    "switch strict +teleop +nav -wrist fault=hardware:base:write:error@3 "
    "fault=controller:nav:activate:error@0 stall hold powercut@2\n"
    "fault controller:arm_ik:update:throw fault=controller:arm_ik:deactivate:hang nowait\n"
    "configure nav fault=controller:nav:configure:hang nowait\n"
    "shutdown fault=controller:teleop:deactivate:hang stall\n");
  // Fails as long as teleop is started while base fails in write, arm_ik faults and nav configures
  const auto fails = [](const Script & s)
  {
    const auto has = [&](auto predicate) { return std::any_of(s.begin(), s.end(), predicate); };
    return has(
             [](const Step & step)
             {
               const auto * sw = std::get_if<SwitchStep>(&step);
               return sw &&
                      std::find(sw->activate.begin(), sw->activate.end(), "teleop") !=
                        sw->activate.end() &&
                      std::any_of(
                        sw->faults.begin(), sw->faults.end(),
                        [](const TimedFault & t) { return t.fault.name == "base"; });
             }) &&
           has([](const Step & step) { return std::holds_alternative<FaultStep>(step); }) &&
           has(
             [](const Step & step)
             {
               const auto * l = std::get_if<LifecycleStep>(&step);
               return l && l->action == LifecycleAction::Configure;
             });
  };
  EXPECT_EQ(
    "switch strict +teleop fault=hardware:base:write:error@3\n"
    "fault controller:arm_ik:update:throw\n"
    "configure nav\n",
    to_text(shrink(script, fails, 1000, seconds(60))));
}

TEST(Shrink, simplifies_again_when_a_later_step_made_room)
{
  const Script script = parse_script(
    "switch strict +teleop fault=controller:teleop:activate:error@0\n"
    "switch strict +nav stall\n");
  // Both steps are needed, and the fault of the first only while the second stalls
  const auto fails = [](const Script & s)
  {
    const auto * teleop = s.empty() ? nullptr : std::get_if<SwitchStep>(&s.front());
    const auto * nav = s.size() < 2 ? nullptr : std::get_if<SwitchStep>(&s.back());
    return teleop && nav && teleop->activate == std::vector<std::string>{"teleop"} &&
           nav->activate == std::vector<std::string>{"nav"} &&
           (!teleop->faults.empty() || !nav->stall);
  };
  EXPECT_EQ(
    "switch strict +teleop\nswitch strict +nav\n",
    to_text(shrink(script, fails, 1000, seconds(60))));
}

TEST(Shrink, signature_keeps_the_message_without_the_script_line_numbers_and_name_lists)
{
  const std::string failure =
    "[gate] [event:inactive_writer] step 3 'switch strict +teleop -nav stall': hardware base: "
    "value written in wl/velocity concerning nav (cycle 412 to 418, 7 times)";
  const std::string signature = failure_signature(failure);
  EXPECT_EQ(
    "[gate] [event:inactive_writer] step: hardware base: value written in wl/velocity concerning "
    "nav (cycle  to ,  times)",
    signature);
  EXPECT_EQ(
    signature, failure_signature(
                 "[gate] [event:inactive_writer] step 1 'switch strict +teleop' then 'recover "
                 "arm': hardware base: value written in wl/velocity concerning nav (cycle 9 to "
                 "10, 2 times)"));
  EXPECT_NE(
    signature, failure_signature(
                 "[gate] [event:inactive_writer] step 3 'switch strict +teleop -nav stall': "
                 "hardware arm: value written in a1/velocity concerning nav (cycle 412 to 418, 7 "
                 "times)"));
  EXPECT_NE(
    failure_signature("[gate] [event:inactive_writer] setup: hardware arm: a1/velocity"),
    failure_signature("[gate] [event:inactive_writer] setup: hardware arm: a3/velocity"));
  EXPECT_EQ(
    "[provisional] [outcome] setup: rejected, but deactivated [] beyond the request",
    failure_signature(
      "[provisional] [outcome] setup: rejected, but deactivated [arm_pos nav] beyond the request"));
  EXPECT_EQ(
    "[provisional] [outcome] setup: accepted, but  is active",
    failure_signature("[provisional] [outcome] setup: accepted, but 2 is active"));
}

TEST(Shrink, reports_the_failure_before_shrinking)
{
  const Script script(8, LifecycleStep{LifecycleAction::Load, "nav", std::nullopt, std::nullopt});
  testing::TestPartResultArray reported;
  int reported_while_shrinking = -1;
  {
    const testing::ScopedFakeTestPartResultReporter intercept(
      testing::ScopedFakeTestPartResultReporter::INTERCEPT_ONLY_CURRENT_THREAD, &reported);
    report_and_shrink(
      "[gate] [exception] step 3 'load nav'", script,
      [&](const Script &)
      {
        reported_while_shrinking = reported.size();
        return true;
      },
      100, seconds(60));
  }
  EXPECT_EQ(1, reported_while_shrinking);
  ASSERT_EQ(2, reported.size());
  EXPECT_NE(
    std::string::npos,
    std::string(reported.GetTestPartResult(0).message()).find("[gate] [exception] step 3"));
  EXPECT_NE(
    std::string::npos, std::string(reported.GetTestPartResult(1).message()).find("Shrunk script"));
}

}  // namespace cm_test_suite
