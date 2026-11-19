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

#include "cm_test_suite/script.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>
#include <utility>

#include "overloaded.hpp"

namespace cm_test_suite
{
namespace
{
constexpr std::array STRICTNESS_NAMES = {"strict", "auto", "force_auto"};
constexpr std::array ACTION_NAMES = {"load", "configure", "cleanup", "unload"};
constexpr std::array PACE_NAMES = {"off", "steady", "jitter", "stall", "long_stall"};

std::vector<std::string> split(std::string_view text, char separator)
{
  std::vector<std::string> parts;
  size_t start = 0;
  while (start <= text.size())
  {
    const size_t end = std::min(text.find(separator, start), text.size());
    if (end > start)
    {
      parts.emplace_back(text.substr(start, end - start));
    }
    start = end + 1;
  }
  return parts;
}

template <typename Names>
size_t index_of(const Names & names, std::string_view text, std::string_view what)
{
  for (size_t i = 0; i < names.size(); ++i)
  {
    if (text == names[i])
    {
      return i;
    }
  }
  throw std::invalid_argument("Unknown " + std::string(what) + " '" + std::string(text) + "'");
}

bool starts_with(const std::string & word, std::string_view prefix)
{
  return word.rfind(prefix, 0) == 0;
}

FaultSpec parse_fault_spec(std::string_view text)
{
  const auto parts = split(text, ':');
  if (parts.size() != 4)
  {
    throw std::invalid_argument("Fault must be kind:name:hook:fault, got " + std::string(text));
  }
  const TargetKind kind =
    index_of(std::array{"controller", "hardware"}, parts[0], "target kind") == 0
      ? TargetKind::Controller
      : TargetKind::Hardware;
  const bool known = kind == TargetKind::Controller ? rig::controller_index(parts[1]).has_value()
                                                    : rig::hardware_index(parts[1]).has_value();
  const auto hook = parse_hook(parts[2]);
  const auto fault = parse_fault(parts[3]);
  if (!known || !hook || !fault || !applicable(FaultSpec{kind, parts[1], *hook, *fault}))
  {
    throw std::invalid_argument("Invalid fault " + std::string(text));
  }
  return {kind, parts[1], *hook, *fault};
}

/// Parses "<value>@<cycles>".
std::pair<std::string, int> timed(const std::string & text)
{
  const auto at = text.find('@');
  if (at == std::string::npos)
  {
    throw std::invalid_argument("Expected <value>@<cycles>, got " + text);
  }
  const int cycles = std::stoi(text.substr(at + 1));
  if (cycles < 0)
  {
    throw std::invalid_argument("Negative cycles in " + text);
  }
  return {text.substr(0, at), cycles};
}

bool parse_expect(const std::string & word)
{
  return index_of(std::array{"reject", "accept"}, word.substr(7), "expectation") == 1;
}

void require_controller(const std::string & name)
{
  if (!rig::controller_index(name))
  {
    throw std::invalid_argument("Unknown controller " + name);
  }
}

SwitchStep parse_switch(const std::vector<std::string> & words)
{
  SwitchStep step;
  step.strictness = static_cast<Strictness>(index_of(STRICTNESS_NAMES, words.at(1), "strictness"));
  for (size_t i = 2; i < words.size(); ++i)
  {
    const auto & word = words[i];
    if (word == "stall")
    {
      step.stall = true;
    }
    else if (word == "hold")
    {
      step.hold = true;
    }
    else if (starts_with(word, "fault="))
    {
      const auto [spec, cycles] = timed(word.substr(6));
      step.faults.push_back({parse_fault_spec(spec), cycles});
    }
    else if (starts_with(word, "powercut@"))
    {
      step.power_cut = timed(word).second;
    }
    else if (starts_with(word, "expect="))
    {
      step.expect_accepted = parse_expect(word);
    }
    else if (word[0] == '+' || word[0] == '-')
    {
      const std::string name = word.substr(1);
      require_controller(name);
      (word[0] == '+' ? step.activate : step.deactivate).push_back(name);
    }
    else
    {
      throw std::invalid_argument("Unexpected switch argument " + word);
    }
  }
  return step;
}

/// Options after the main argument of lifecycle and recover steps.
void parse_options(
  const std::vector<std::string> & words, size_t first, std::optional<FaultSpec> & fault,
  std::optional<bool> & expect)
{
  for (size_t i = first; i < words.size(); ++i)
  {
    if (starts_with(words[i], "fault="))
    {
      fault = parse_fault_spec(words[i].substr(6));
    }
    else if (starts_with(words[i], "expect="))
    {
      expect = parse_expect(words[i]);
    }
    else
    {
      throw std::invalid_argument("Unexpected argument " + words[i]);
    }
  }
}

Step parse_step(const std::vector<std::string> & words)
{
  const std::string & verb = words[0];
  if (verb == "switch" && words.size() >= 2)
  {
    return parse_switch(words);
  }
  if (verb == "shutdown")
  {
    ShutdownStep step;
    std::vector<std::string> options;
    for (const auto & word : words)
    {
      if (word == "stall")
      {
        step.stall = true;
      }
      else
      {
        options.push_back(word);
      }
    }
    std::optional<bool> expect;
    parse_options(options, 1, step.fault, expect);
    if (expect)
    {
      throw std::invalid_argument("A shutdown has no answer to expect");
    }
    return step;
  }
  if (verb == "slowio" && words.size() <= 2)
  {
    const auto pace = words.size() == 1
                        ? SlowIo::Steady
                        : static_cast<SlowIo>(index_of(PACE_NAMES, words[1], "slow I/O pace"));
    if (pace == SlowIo::Off)
    {
      throw std::invalid_argument("A slow I/O step needs a pace");
    }
    return SlowIoStep{pace};
  }
  if (words.size() < 2)
  {
    throw std::invalid_argument("Missing argument");
  }
  if (verb == "fault")
  {
    FaultStep step{parse_fault_spec(words[1]), {}, true};
    for (size_t i = 2; i < words.size(); ++i)
    {
      if (words[i] == "nowait")
      {
        step.wait = false;
      }
      else if (starts_with(words[i], "fault="))
      {
        step.handling.push_back(parse_fault_spec(words[i].substr(6)));
      }
      else
      {
        throw std::invalid_argument("Unexpected fault argument " + words[i]);
      }
    }
    return step;
  }
  if (verb == "powercut")
  {
    const bool cut = index_of(std::array{"off", "on"}, words[1], "power cut state") == 1;
    const bool nowait = cut && words.size() == 3 && words[2] == "nowait";
    if (words.size() > (nowait ? 3 : 2))
    {
      throw std::invalid_argument("Expected powercut on [nowait] | powercut off");
    }
    return PowerStep{cut, !nowait};
  }
  if (verb == "recover")
  {
    if (!rig::hardware_index(words[1]))
    {
      throw std::invalid_argument("Unknown hardware " + words[1]);
    }
    RecoverStep step{words[1], std::nullopt, std::nullopt};
    parse_options(words, 2, step.fault, step.expect_accepted);
    return step;
  }
  const auto action = static_cast<LifecycleAction>(index_of(ACTION_NAMES, verb, "step"));
  require_controller(words[1]);
  LifecycleStep step{action, words[1], std::nullopt, std::nullopt};
  std::vector<std::string> options;
  for (const auto & word : words)
  {
    if (word == "nowait")
    {
      step.wait = false;
    }
    else
    {
      options.push_back(word);
    }
  }
  parse_options(options, 2, step.fault, step.expect_accepted);
  return step;
}

std::string options_text(const std::optional<FaultSpec> & fault, const std::optional<bool> & expect)
{
  std::string text;
  if (fault)
  {
    text += " fault=" + to_text(*fault);
  }
  if (expect)
  {
    text += *expect ? " expect=accept" : " expect=reject";
  }
  return text;
}
}  // namespace

const char * to_string(Strictness strictness)
{
  return STRICTNESS_NAMES.at(static_cast<size_t>(strictness));
}

bool applicable(const FaultSpec & fault)
{
  const bool io = fault.hook == Hook::Read || fault.hook == Hook::Write;
  if (fault.kind == TargetKind::Hardware && io && fault.fault == Fault::Hang)
  {
    const auto index = rig::hardware_index(fault.name);
    return index && rig::hardware()[*index].is_async;
  }
  return applicable(fault.kind, fault.hook, fault.fault);
}

std::string to_text(const FaultSpec & fault)
{
  return std::string(fault.kind == TargetKind::Controller ? "controller" : "hardware") + ":" +
         fault.name + ":" + to_string(fault.hook) + ":" + to_string(fault.fault);
}

std::string to_text(const Step & step)
{
  return std::visit(
    Overloaded{
      [](const SwitchStep & s)
      {
        std::string text = std::string("switch ") + to_string(s.strictness);
        for (const auto & name : s.activate)
        {
          text += " +" + name;
        }
        for (const auto & name : s.deactivate)
        {
          text += " -" + name;
        }
        for (const auto & timed_fault : s.faults)
        {
          text +=
            " fault=" + to_text(timed_fault.fault) + "@" + std::to_string(timed_fault.after_cycles);
        }
        if (s.stall)
        {
          text += " stall";
        }
        if (s.hold)
        {
          text += " hold";
        }
        if (s.power_cut)
        {
          text += " powercut@" + std::to_string(*s.power_cut);
        }
        return text + options_text(std::nullopt, s.expect_accepted);
      },
      [](const FaultStep & s)
      {
        std::string text = "fault " + to_text(s.fault);
        for (const auto & handling : s.handling)
        {
          text += " fault=" + to_text(handling);
        }
        return text + (s.wait ? "" : " nowait");
      },
      [](const LifecycleStep & s)
      {
        return std::string(ACTION_NAMES.at(static_cast<size_t>(s.action))) + " " + s.controller +
               options_text(s.fault, s.expect_accepted) + (s.wait ? "" : " nowait");
      },
      [](const RecoverStep & s)
      { return "recover " + s.hardware + options_text(s.fault, s.expect_accepted); },
      [](const PowerStep & s)
      { return std::string("powercut ") + (s.cut ? "on" : "off") + (s.wait ? "" : " nowait"); },
      [](const SlowIoStep & s)
      {
        return s.pace == SlowIo::Steady
                 ? std::string("slowio")
                 : "slowio " + std::string(PACE_NAMES.at(static_cast<size_t>(s.pace)));
      },
      [](const ShutdownStep & s)
      { return "shutdown" + options_text(s.fault, std::nullopt) + (s.stall ? " stall" : ""); }},
    step);
}

std::string to_text(const Script & script)
{
  std::string text;
  for (const auto & step : script)
  {
    text += to_text(step) + "\n";
  }
  return text;
}

Script parse_script(std::string_view text)
{
  Script script;
  for (const auto & line : split(text, '\n'))
  {
    const auto words = split(line, ' ');
    if (words.empty() || words[0][0] == '#')
    {
      continue;
    }
    if (!script.empty() && std::holds_alternative<ShutdownStep>(script.back()))
    {
      throw std::invalid_argument("Line '" + line + "': shutdown must be the last step");
    }
    try
    {
      script.push_back(parse_step(words));
    }
    catch (const std::exception & e)
    {
      throw std::invalid_argument("Line '" + line + "': " + e.what());
    }
  }
  return script;
}

}  // namespace cm_test_suite
