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
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <variant>

#include "cm_test_suite/script.hpp"

namespace cm_test_suite
{
TEST(Script, parses_every_step_kind)
{
  const Script script = parse_script(
    "# comment\n"
    "switch force_auto +teleop -nav fault=hardware:base:write:error@3 stall\n"
    "fault controller:arm_ik:update:throw\n"
    "\n"
    "unload nav\n"
    "recover arm\n"
    "powercut on\n");
  ASSERT_EQ(5u, script.size());
  const auto & s = std::get<SwitchStep>(script[0]);
  EXPECT_EQ(Strictness::ForceAuto, s.strictness);
  EXPECT_EQ(std::vector<std::string>{"teleop"}, s.activate);
  EXPECT_EQ(std::vector<std::string>{"nav"}, s.deactivate);
  ASSERT_EQ(1u, s.faults.size());
  EXPECT_EQ("base", s.faults[0].fault.name);
  EXPECT_EQ(Hook::Write, s.faults[0].fault.hook);
  EXPECT_EQ(3, s.faults[0].after_cycles);
  EXPECT_TRUE(s.stall);
  EXPECT_EQ(LifecycleAction::Unload, std::get<LifecycleStep>(script[2]).action);
  EXPECT_TRUE(std::get<PowerStep>(script[4]).cut);
}

TEST(Script, parses_faults_expectations_and_shutdown)
{
  const Script script = parse_script(
    "switch auto +teleop fault=controller:wrist:activate:error@0 "
    "fault=hardware:arm:write:error@2 powercut@1 hold expect=reject\n"
    "configure nav fault=controller:nav:configure:throw expect=reject nowait\n"
    "recover arm fault=hardware:arm:activate:failure\n"
    "powercut on nowait\n"
    "shutdown fault=controller:arm_vel:deactivate:hang stall\n");
  const Script fault_step = parse_script(
    "fault controller:wrist:update:error fault=controller:wrist:deactivate:late "
    "fault=controller:arm_vel:deactivate:late nowait\n");
  const auto & f = std::get<FaultStep>(fault_step[0]);
  EXPECT_FALSE(f.wait);
  ASSERT_EQ(2u, f.handling.size());
  EXPECT_EQ(Fault::Late, f.handling[0].fault);
  EXPECT_EQ("arm_vel", f.handling[1].name);
  EXPECT_EQ(
    "fault controller:wrist:update:error fault=controller:wrist:deactivate:late "
    "fault=controller:arm_vel:deactivate:late nowait\n",
    to_text(fault_step));
  ASSERT_EQ(5u, script.size());
  const auto & s = std::get<SwitchStep>(script[0]);
  EXPECT_EQ(2u, s.faults.size());
  EXPECT_EQ(1, *s.power_cut);
  EXPECT_TRUE(s.hold);
  EXPECT_EQ(false, *s.expect_accepted);
  EXPECT_TRUE(std::get<LifecycleStep>(script[1]).fault.has_value());
  EXPECT_FALSE(std::get<LifecycleStep>(script[1]).wait);
  EXPECT_FALSE(std::get<PowerStep>(script[3]).wait);
  EXPECT_TRUE(std::get<ShutdownStep>(script[4]).fault.has_value());
  EXPECT_TRUE(std::get<ShutdownStep>(script[4]).stall);
  EXPECT_THROW(parse_script("shutdown\nload nav\n"), std::invalid_argument);
}

TEST(Script, rejects_invalid_lines)
{
  EXPECT_THROW(parse_script("switch strict +no_such_controller"), std::invalid_argument);
  EXPECT_THROW(parse_script("fault hardware:arm:update:error"), std::invalid_argument);
  EXPECT_THROW(parse_script("switch sometimes +nav"), std::invalid_argument);
  EXPECT_THROW(parse_script("recover"), std::invalid_argument);
  EXPECT_THROW(parse_script("powercut off nowait"), std::invalid_argument);
  EXPECT_THROW(parse_script("slowio off"), std::invalid_argument);
  EXPECT_THROW(parse_script("shutdown expect=accept"), std::invalid_argument)
    << "a shutdown has no answer to expect";
  EXPECT_THROW(parse_script("switch strict +nav powercut@-1"), std::invalid_argument);
  EXPECT_THROW(
    parse_script("switch strict +nav fault=controller:nav:activate:error@-2"),
    std::invalid_argument);
}

TEST(Script, only_async_hardware_hangs_in_read_or_write)
{
  EXPECT_NO_THROW(parse_script("fault hardware:base:write:hang"));
  EXPECT_NO_THROW(parse_script("fault hardware:imu:read:hang"));
  EXPECT_THROW(parse_script("fault hardware:arm:write:hang"), std::invalid_argument);
  EXPECT_THROW(parse_script("fault hardware:flipper:read:hang"), std::invalid_argument);
}

TEST(Script, generated_scripts_survive_a_text_round_trip)
{
  Capabilities all;
  all.auto_switch = all.force_auto_switch = all.recover_hardware = true;
  const Script script = generate_script(7, 300, all);
  const std::string text = to_text(script);
  EXPECT_EQ(text, to_text(parse_script(text)));
}

TEST(Script, slow_mode_switches_hit_sync_and_async_hardware)
{
  Capabilities all;
  all.auto_switch = all.force_auto_switch = all.recover_hardware = true;
  size_t slow_sync = 0;
  size_t slow_async = 0;
  for (uint64_t seed = 0; seed < 50; ++seed)
  {
    for (const auto & step : generate_script(seed, 100, all))
    {
      const auto * s = std::get_if<SwitchStep>(&step);
      for (const auto & timed : s ? s->faults : std::vector<TimedFault>{})
      {
        const auto & fault = timed.fault;
        const bool switch_call = fault.hook == Hook::Prepare || fault.hook == Hook::Perform;
        if (switch_call && fault.fault == Fault::Delay)
        {
          const bool async = rig::hardware()[*rig::hardware_index(fault.name)].is_async;
          ++(async ? slow_async : slow_sync);
        }
      }
    }
  }
  EXPECT_GT(slow_sync, 0u);
  EXPECT_GT(slow_async, 0u);
}

TEST(Script, a_stalled_switch_cuts_the_power_before_its_switch_returned)
{
  Capabilities all;
  all.auto_switch = all.force_auto_switch = all.recover_hardware = true;
  size_t stalled_cuts = 0;
  for (uint64_t seed = 0; seed < 1000; ++seed)
  {
    for (const auto & step : generate_script(seed, 100, all))
    {
      const auto * s = std::get_if<SwitchStep>(&step);
      if (s && s->stall && s->power_cut)
      {
        EXPECT_LE(*s->power_cut, 1) << to_text(step);
        ++stalled_cuts;
      }
    }
  }
  EXPECT_GT(stalled_cuts, 2u);
}

TEST(Script, switches_activate_quiet_controllers)
{
  Capabilities all;
  all.auto_switch = all.force_auto_switch = all.recover_hardware = true;
  size_t quiet = 0;
  for (uint64_t seed = 0; seed < 20; ++seed)
  {
    for (const auto & step : generate_script(seed, 100, all))
    {
      const auto * s = std::get_if<SwitchStep>(&step);
      for (const auto & timed : s ? s->faults : std::vector<TimedFault>{})
      {
        quiet += timed.fault.fault == Fault::Quiet && timed.fault.hook == Hook::Activate;
      }
    }
  }
  EXPECT_GT(quiet, 0u);
}

TEST(Script, switch_deadlines_pass_during_a_slow_prepare_or_perform)
{
  Capabilities all;
  all.auto_switch = all.force_auto_switch = all.recover_hardware = true;
  size_t deadline_switches = 0;
  for (uint64_t seed = 0; seed < 20; ++seed)
  {
    for (const auto & step : generate_script(seed, 100, all))
    {
      const auto * s = std::get_if<SwitchStep>(&step);
      if (!s)
      {
        continue;
      }
      const auto has = [&](auto predicate)
      {
        return std::any_of(
          s->faults.begin(), s->faults.end(),
          [&](const TimedFault & t) { return predicate(t.fault); });
      };
      const bool late = has([](const FaultSpec & f) { return f.fault == Fault::Late; });
      const bool slow_call = has(
        [](const FaultSpec & f)
        {
          return f.fault == Fault::Delay && (f.hook == Hook::Perform || f.hook == Hook::Prepare);
        });
      deadline_switches += late && slow_call;
    }
  }
  EXPECT_GT(deadline_switches, 5u);
}

TEST(Script, switches_arrive_while_a_fault_is_handled_slowly)
{
  Capabilities all;
  all.auto_switch = all.force_auto_switch = all.recover_hardware = true;
  size_t meetings = 0;
  for (uint64_t seed = 0; seed < 20; ++seed)
  {
    const Script script = generate_script(seed, 100, all);
    for (size_t i = 0; i + 1 < script.size(); ++i)
    {
      const auto * fault = std::get_if<FaultStep>(&script[i]);
      if (fault && !fault->wait)
      {
        ASSERT_FALSE(fault->handling.empty());
        EXPECT_TRUE(std::holds_alternative<SwitchStep>(script[i + 1])) << to_text(script[i + 1]);
        ++meetings;
      }
    }
  }
  EXPECT_GT(meetings, 5u);
}

TEST(Script, async_hardware_is_sometimes_slow_jittery_stalled_or_hung)
{
  Capabilities all;
  all.auto_switch = all.force_auto_switch = all.recover_hardware = true;
  std::map<SlowIo, size_t> paces;
  size_t io_hangs = 0;
  for (uint64_t seed = 0; seed < 50; ++seed)
  {
    for (const auto & step : generate_script(seed, 100, all))
    {
      if (const auto * s = std::get_if<SlowIoStep>(&step))
      {
        ++paces[s->pace];
      }
      const auto * fault = std::get_if<FaultStep>(&step);
      const bool io =
        fault && (fault->fault.hook == Hook::Read || fault->fault.hook == Hook::Write);
      if (io && fault->fault.fault == Fault::Hang)
      {
        EXPECT_TRUE(rig::hardware()[*rig::hardware_index(fault->fault.name)].is_async);
        ++io_hangs;
      }
    }
  }
  for (const auto pace : {SlowIo::Steady, SlowIo::Jitter, SlowIo::Stall, SlowIo::LongStall})
  {
    EXPECT_GT(paces[pace], 5u) << static_cast<int>(pace);
  }
  EXPECT_GT(io_hangs, 0u);
  for (const char * text : {"slowio\n", "slowio jitter\n", "slowio stall\n", "slowio long_stall\n"})
  {
    EXPECT_EQ(text, to_text(parse_script(text)));
  }
}

TEST(Script, generated_distribution_over_1000_seeds)
{
  Capabilities all;
  all.auto_switch = all.force_auto_switch = all.recover_hardware = true;
  size_t steps = 0;
  size_t unpowered = 0;
  size_t holds = 0;
  size_t unload_shutdown_faults = 0;
  size_t shutdown_step_faults = 0;
  size_t lifecycle_nowait = 0;
  size_t slow_handling_waited = 0;
  size_t auto_started_activate_faults = 0;
  size_t rollback_deactivate_faults = 0;
  size_t conflict_deactivate_faults = 0;
  for (uint64_t seed = 0; seed < 1000; ++seed)
  {
    bool cut = false;
    size_t cut_for = 0;
    bool meets_fault_handling = false;
    for (const auto & step : generate_script(seed, 100, all))
    {
      ++steps;
      const auto * power = std::get_if<PowerStep>(&step);
      const auto * s = std::get_if<SwitchStep>(&step);
      const auto * f = std::get_if<FaultStep>(&step);
      if (power && !power->cut)
      {
        EXPECT_TRUE(cut) << "seed " << seed;
        EXPECT_GE(cut_for, 1u) << "seed " << seed;
        cut = false;
        continue;
      }
      // Counted like the runner, a step that starts or ends unpowered
      const bool was_cut = cut;
      // A switch meeting a slow fault handling belongs to its fault step
      const bool belongs_to_fault_step = std::exchange(meets_fault_handling, f && !f->wait);
      cut_for += cut && !belongs_to_fault_step;
      if (!std::holds_alternative<ShutdownStep>(step))
      {
        EXPECT_LE(cut_for, 3u) << "seed " << seed;
      }
      if ((power && power->cut) || (s && s->power_cut))
      {
        cut = true;
        cut_for = 0;
      }
      unpowered += was_cut || cut;
      if (s)
      {
        holds += s->hold;
        EXPECT_FALSE(s->hold && s->stall) << to_text(step);
        rig::Names started(s->activate.begin(), s->activate.end());
        if (s->strictness != Strictness::Strict)
        {
          started.merge(rig::closure(started, rig::providers));
        }
        rig::Names touched = started;
        touched.insert(s->deactivate.begin(), s->deactivate.end());
        if (s->strictness == Strictness::ForceAuto)
        {
          for (const auto & name : started)
          {
            touched.merge(rig::command_conflicts(name));
          }
        }
        for (const auto & timed : s->faults)
        {
          const auto & fault = timed.fault;
          if (fault.kind != TargetKind::Controller || fault.hook == Hook::Update)
          {
            continue;
          }
          if (fault.hook == Hook::ChainedMode)
          {
            const auto writers = rig::reference_writers(fault.name);
            EXPECT_TRUE(
              std::any_of(
                touched.begin(), touched.end(),
                [&](const std::string & name) { return writers.count(name) > 0; }))
              << to_text(step);
            continue;
          }
          EXPECT_EQ(1u, (fault.hook == Hook::Activate ? started : touched).count(fault.name))
            << to_text(step);
          const auto requested = [&](const std::vector<std::string> & names)
          { return std::find(names.begin(), names.end(), fault.name) != names.end(); };
          auto_started_activate_faults += fault.hook == Hook::Activate && !requested(s->activate);
          if (fault.hook == Hook::Deactivate && !requested(s->deactivate))
          {
            ++(started.count(fault.name) ? rollback_deactivate_faults : conflict_deactivate_faults);
          }
        }
      }
      if (const auto * l = std::get_if<LifecycleStep>(&step))
      {
        unload_shutdown_faults += l->fault && l->fault->hook == Hook::Shutdown;
        if (!l->wait)
        {
          ++lifecycle_nowait;
          EXPECT_TRUE(l->fault && l->fault->fault == Fault::Hang) << to_text(step);
        }
      }
      if (f)
      {
        slow_handling_waited += f->wait && !f->handling.empty();
      }
      if (const auto * d = std::get_if<ShutdownStep>(&step))
      {
        shutdown_step_faults += d->fault && d->fault->hook == Hook::Shutdown;
      }
    }
  }
  const double unpowered_share = static_cast<double>(unpowered) / static_cast<double>(steps);
  EXPECT_GT(unpowered_share, 0.10);
  EXPECT_LT(unpowered_share, 0.20);
  EXPECT_GT(holds, 100u);
  EXPECT_GT(unload_shutdown_faults, 100u);
  EXPECT_GT(shutdown_step_faults, 10u);
  EXPECT_GT(lifecycle_nowait, 10u);
  EXPECT_GT(slow_handling_waited, 100u);
  EXPECT_GT(auto_started_activate_faults, 100u);
  EXPECT_GT(rollback_deactivate_faults, 300u);
  EXPECT_GT(conflict_deactivate_faults, 30u);
}

TEST(Script, a_seed_gives_the_same_script_with_every_standard_library)
{
  Capabilities all;
  all.auto_switch = all.force_auto_switch = all.recover_hardware = true;
  // Pinned so a draw that depends on the standard library, such as a distribution in below(),
  // fails here. Update it only together with an intended change of the generator.
  EXPECT_EQ(
    "switch strict -odom_pub fault=controller:odom_pub:error:failure@4\n"
    "switch auto +whole_body fault=hardware:base:prepare:error@4 powercut@4\n"
    "fault controller:arm_pos:update:error\n"
    "switch strict +monitor +arm_pos -arm_pos fault=hardware:arm:deactivate:error@2\n"
    "switch auto +arm_ik\n"
    "powercut off\n",
    to_text(generate_script(1, 6, all)));
  // Seeds 16 to 18 reach every draw site, pinned as an FNV-1a digest of their text
  uint64_t digest = 0xcbf29ce484222325;
  for (uint64_t seed = 16; seed < 19; ++seed)
  {
    for (const unsigned char c : to_text(generate_script(seed, 100, all)))
    {
      digest = (digest ^ c) * 0x100000001b3;
    }
  }
  EXPECT_EQ(9757431269405458382u, digest);
}

TEST(Script, generation_is_deterministic_and_respects_capabilities)
{
  const Capabilities strict_only;
  const Script script = generate_script(3, 300, strict_only);
  EXPECT_EQ(to_text(script), to_text(generate_script(3, 300, strict_only)));
  for (const auto & step : script)
  {
    EXPECT_FALSE(std::holds_alternative<PowerStep>(step));
    EXPECT_FALSE(std::holds_alternative<RecoverStep>(step));
    if (const auto * s = std::get_if<SwitchStep>(&step))
    {
      EXPECT_EQ(Strictness::Strict, s->strictness);
    }
  }
  Capabilities recover_only;
  recover_only.recover_hardware = true;
  size_t power_steps = 0;
  for (const auto & step : generate_script(3, 300, recover_only))
  {
    power_steps += std::holds_alternative<PowerStep>(step);
    if (const auto * s = std::get_if<SwitchStep>(&step))
    {
      EXPECT_EQ(Strictness::Strict, s->strictness);
    }
  }
  EXPECT_GT(power_steps, 0u) << "the power recovery needs no AUTO switch";
}

TEST(Script, acceptance_scripts_parse)
{
  size_t count = 0;
  for (const auto & entry : std::filesystem::directory_iterator(SCRIPTS_DIR))
  {
    std::ifstream file(entry.path());
    std::stringstream text;
    text << file.rdbuf();
    EXPECT_NO_THROW(parse_script(text.str())) << entry.path();
    ++count;
  }
  EXPECT_GT(count, 0u);
}

}  // namespace cm_test_suite
