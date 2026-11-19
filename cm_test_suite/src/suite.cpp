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

#include "cm_test_suite/suite.hpp"

#include <gtest/gtest.h>

#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_path.hpp"
#include "cm_test_suite/runner.hpp"
#include "cm_test_suite/shrink.hpp"
#include "cm_test_suite/trace.hpp"

namespace cm_test_suite
{
namespace
{
constexpr size_t LOG_TAIL = 20;
// A passing candidate costs two runs, the time budget still caps the shrink
constexpr size_t SHRINK_RUNS = 200;

uint64_t env_number(const char * name, uint64_t fallback)
{
  const char * value = std::getenv(name);
  return value ? std::stoull(value) : fallback;
}

std::set<std::string> skipped_checks()
{
  std::set<std::string> skipped;
  const char * value = std::getenv("CM_FUZZ_SKIP");
  std::stringstream stream(value ? value : "");
  for (std::string item; std::getline(stream, item, ',');)
  {
    if (!item.empty())
    {
      skipped.insert(item);
    }
  }
  return skipped;
}

std::string read_file(const std::filesystem::path & path)
{
  std::ifstream file(path);
  if (!file)
  {
    throw std::runtime_error("Cannot read " + path.string());
  }
  std::stringstream text;
  text << file.rdbuf();
  return text.str();
}

/// The first capability the script needs but the controller manager lacks.
std::optional<std::string> missing_capability(const Script & script, const Capabilities & has)
{
  for (const auto & step : script)
  {
    if (const auto * s = std::get_if<SwitchStep>(&step))
    {
      if (s->strictness == Strictness::Auto && !has.auto_switch)
      {
        return "auto switch";
      }
      if (s->strictness == Strictness::ForceAuto && !has.force_auto_switch)
      {
        return "force auto switch";
      }
    }
    if (std::holds_alternative<RecoverStep>(step) && !has.recover_hardware)
    {
      return "hardware recovery";
    }
    const auto * power = std::get_if<PowerStep>(&step);
    if (power && !power->cut && !has.recover_hardware)
    {
      return "power recovery";
    }
  }
  return std::nullopt;
}

std::string signature(const RunResult & result)
{
  return result.passed() ? "" : failure_signature(result.failures.front());
}

/// The run statistics as gtest properties, whose names must be XML attribute names.
std::vector<std::pair<std::string, int64_t>> properties(const RunStats & stats)
{
  const auto name = [](std::string text)
  {
    std::replace_if(text.begin(), text.end(), [](char c) { return c == ' ' || c == ':'; }, '_');
    return text;
  };
  std::vector<std::pair<std::string, int64_t>> list;
  for (const auto & [kind, answers] : stats.answers)
  {
    list.emplace_back(name(kind) + "_accepted", answers.accepted);
    list.emplace_back(name(kind) + "_rejected", answers.rejected);
  }
  for (const auto & [kind, count] : stats.faults_armed)
  {
    list.emplace_back("fault_" + name(kind) + "_armed", count);
    const auto taken = stats.faults_taken.find(kind);
    list.emplace_back(
      "fault_" + name(kind) + "_taken", taken == stats.faults_taken.end() ? 0 : taken->second);
  }
  list.emplace_back("unpowered_steps", stats.unpowered_steps);
  list.emplace_back("settle_deadline_hits", stats.settle_deadline_hits);
  list.emplace_back(
    "longest_cycle_us",
    std::chrono::duration_cast<std::chrono::microseconds>(stats.longest_cycle).count());
  return list;
}

void record_stats(const RunStats & stats)
{
  std::string line = "stats:";
  for (const auto & [key, value] : properties(stats))
  {
    testing::Test::RecordProperty(key, std::to_string(value));
    line += " " + key + "=" + std::to_string(value);
  }
  if (env_number("CM_FUZZ_STATS", 0) != 0)
  {
    std::printf("%s\n", line.c_str());
  }
}

std::string describe_failure(const RunResult & result)
{
  std::string text;
  for (const auto & failure : result.failures)
  {
    text += "\n  " + failure;
  }
  text += "\nLast steps:";
  const size_t first = result.log.size() > LOG_TAIL ? result.log.size() - LOG_TAIL : 0;
  for (size_t i = first; i < result.log.size(); ++i)
  {
    text += "\n  " + result.log[i];
  }
  return text;
}

class ScriptTest : public testing::Test
{
public:
  ScriptTest(CmFactory factory, RunOptions options, std::function<Script()> script, bool shrink)
  : factory_(std::move(factory)),
    options_(std::move(options)),
    script_(std::move(script)),
    shrink_(shrink)
  {
  }

  void TestBody() override
  {
    const Script script = script_();
    if (const auto missing = missing_capability(script, options_.capabilities))
    {
      if (env_number("CM_FUZZ_REQUIRE_ALL", 0) != 0)
      {
        FAIL() << "The controller manager does not support required " << *missing;
      }
      GTEST_SKIP() << "The controller manager does not support " << *missing;
    }
    const RunResult result = run_script(factory_, script, options_);
    record_stats(result.stats);
    if (result.passed())
    {
      return;
    }
    const std::string message = describe_failure(result);
    const std::string failure = signature(result);
    // Setup runs before the first step, so no shorter script changes a setup failure
    const bool setup_failed = failure.find("] setup: ") != std::string::npos;
    if (!shrink_ || env_number("CM_FUZZ_SHRINK", 1) == 0 || setup_failed)
    {
      FAIL() << message;
    }
    report_and_shrink(
      message, script, [&](const Script & candidate)
      { return signature(run_script(factory_, candidate, options_)) == failure; }, SHRINK_RUNS,
      std::chrono::seconds(env_number("CM_FUZZ_SHRINK_SECONDS", 60)));
  }

private:
  CmFactory factory_;
  RunOptions options_;
  std::function<Script()> script_;
  bool shrink_;
};

/// Runs the description check in a child process, so a crash fails this test and nothing else.
class DescriptionTest : public testing::Test
{
public:
  DescriptionTest(CmFactory factory, RunOptions options)
  : factory_(std::move(factory)), options_(std::move(options))
  {
  }

  void TestBody() override
  {
    const std::string findings =
      testing::TempDir() + "cm_test_suite_description_" + std::to_string(::getpid()) + ".txt";
    int status = 0;
    GTEST_FLAG_SET(death_test_style, "threadsafe");
    EXPECT_EXIT(
      {
        const RunResult result =
          run_rejected_description(factory_, rig::urdf_with_duplicate_interfaces(), options_);
        std::ofstream file(findings);
        for (const auto & failure : result.failures)
        {
          file << failure << "\n";
        }
        file.close();
        std::_Exit(0);
      },
      [&](int exit_status)
      {
        status = exit_status;
        return true;
      },
      "");
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0)
    {
      FAIL() << "\n  [gate] [exception] setup: the controller manager crashed on a description "
                "with duplicate interfaces, wait status "
             << status;
    }
    std::ifstream file(findings);
    std::string message;
    for (std::string line; std::getline(file, line);)
    {
      message += "\n  " + line;
    }
    std::remove(findings.c_str());
    if (!message.empty())
    {
      FAIL() << message;
    }
  }

private:
  CmFactory factory_;
  RunOptions options_;
};

void register_test(
  const std::string & suite, const std::string & name, const CmFactory & factory,
  const RunOptions & options, std::function<Script()> script, bool shrink)
{
  testing::RegisterTest(
    suite.c_str(), name.c_str(), nullptr, nullptr, __FILE__, __LINE__,
    [=]() -> testing::Test * { return new ScriptTest(factory, options, script, shrink); });
}

std::vector<std::filesystem::path> acceptance_scripts()
{
  const auto directory = ament_index_cpp::get_package_share_path("cm_test_suite") / "scripts";
  std::vector<std::filesystem::path> paths;
  for (const auto & entry : std::filesystem::directory_iterator(directory))
  {
    if (entry.path().extension() == ".txt")
    {
      paths.push_back(entry.path());
    }
  }
  std::sort(paths.begin(), paths.end());
  return paths;
}
}  // namespace

void register_suite(const std::string & name, const Capabilities & capabilities, CmFactory factory)
{
  const bool complete = capabilities.auto_switch && capabilities.force_auto_switch &&
                        capabilities.startup && capabilities.recover_hardware;
  if (env_number("CM_FUZZ_REQUIRE_ALL", 0) != 0 && !complete)
  {
    testing::RegisterTest(
      (name + "Capabilities").c_str(), "all_required", nullptr, nullptr, __FILE__, __LINE__,
      []() -> testing::Test *
      {
        class Missing : public testing::Test
        {
          void TestBody() override { FAIL() << "Judging requires every capability"; }
        };
        return new Missing;
      });
  }
  trace().set_async_grace_cycles(
    env_number("CM_FUZZ_ASYNC_GRACE_CYCLES", DEFAULT_ASYNC_GRACE_CYCLES));
  RunOptions stepped;
  stepped.capabilities = capabilities;
  stepped.config = default_config();
  stepped.skipped_checks = skipped_checks();
  const char * tier = std::getenv("CM_FUZZ_TIER");
  stepped.gates_only = tier && std::string(tier) == "gates";
  RunOptions threaded = stepped;
  threaded.mode = Mode::Threaded;

  for (const auto & path : acceptance_scripts())
  {
    for (const auto & [mode, options] : {std::pair{"stepped", stepped}, {"threaded", threaded}})
    {
      register_test(
        name + "Acceptance", path.stem().string() + "_" + mode, factory, options,
        [path] { return parse_script(read_file(path)); }, false);
    }
  }
  for (const auto & [mode, options] : {std::pair{"stepped", stepped}, {"threaded", threaded}})
  {
    testing::RegisterTest(
      (name + "Acceptance").c_str(), (std::string("duplicate_interfaces_rejected_") + mode).c_str(),
      nullptr, nullptr, __FILE__, __LINE__, [factory, options = options]() -> testing::Test *
      { return new DescriptionTest(factory, options); });
  }
  if (const char * path = std::getenv("CM_FUZZ_SCRIPT"))
  {
    const std::string file = path;
    register_test(
      name + "Replay", "stepped", factory, stepped,
      [file] { return parse_script(read_file(file)); }, false);
    register_test(
      name + "Replay", "threaded", factory, threaded,
      [file] { return parse_script(read_file(file)); }, false);
  }

  const uint64_t first = env_number("CM_FUZZ_SEED", 0);
  const uint64_t steps = env_number("CM_FUZZ_STEPS", 100);
  const auto add_fuzz = [&](const char * mode, const RunOptions & options, uint64_t seeds)
  {
    for (uint64_t seed = first; seed < first + seeds; ++seed)
    {
      register_test(
        name + "Fuzz", std::string(mode) + "_seed_" + std::to_string(seed), factory, options,
        [seed, steps, capabilities] { return generate_script(seed, steps, capabilities); }, true);
    }
  };
  add_fuzz("stepped", stepped, env_number("CM_FUZZ_SEEDS", 20));
  add_fuzz("threaded", threaded, env_number("CM_FUZZ_THREADED_SEEDS", 5));
}

}  // namespace cm_test_suite
