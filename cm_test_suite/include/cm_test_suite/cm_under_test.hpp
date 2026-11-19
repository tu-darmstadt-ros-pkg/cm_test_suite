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

#ifndef CM_TEST_SUITE__CM_UNDER_TEST_HPP_
#define CM_TEST_SUITE__CM_UNDER_TEST_HPP_

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "cm_test_suite/rig.hpp"
#include "cm_test_suite/types.hpp"

namespace cm_test_suite
{
enum class Strictness
{
  Strict,
  Auto,
  ForceAuto,
};

const char * to_string(Strictness strictness);

struct Capabilities
{
  bool auto_switch = false;
  bool force_auto_switch = false;
  /// Loads and activates the startup profile of the config by itself.
  bool startup = false;
  /// Can bring a faulted hardware component back to active on request. Enables power cuts, whose
  /// recovery needs it.
  bool recover_hardware = false;
};

struct CmConfig
{
  std::string robot_description;
  rig::StartupProfile startup;
  std::chrono::milliseconds switch_timeout{1000};
};

/// What the controller manager reports, as list_controllers and list_hardware_components would.
struct ReportedState
{
  /// Loaded controllers only.
  std::map<std::string, Lifecycle> controllers;
  std::map<std::string, Lifecycle> hardware;
  /// Controllers and hardware shown in a transition state such as deactivating, whose entry above
  /// is meaningless. Allowed while a step runs, never once it settled.
  std::set<std::string> transitioning;
};

/// The adapter a controller manager implementation provides to run the common test suite.
///
/// Threading contract: cycle() is called from one thread at a time. The operations are called
/// from another thread and may block until cycles have run. reported() and activity() may be
/// called from a third thread at any time. Every operation returns whether the controller manager
/// accepted it.
class CmUnderTest
{
public:
  virtual ~CmUnderTest() = default;

  /// One control cycle: read, update and write with a fixed period.
  virtual void cycle() = 0;

  virtual bool load(const std::string & controller) = 0;
  virtual bool configure(const std::string & controller) = 0;
  virtual bool cleanup(const std::string & controller) = 0;
  virtual bool unload(const std::string & controller) = 0;
  virtual bool switch_controllers(
    const std::vector<std::string> & activate, const std::vector<std::string> & deactivate,
    Strictness strictness) = 0;

  /// Only called with Capabilities::recover_hardware, otherwise it may throw.
  virtual bool recover_hardware(const std::string & hardware) = 0;

  virtual ReportedState reported() = 0;
  /// The state in the last message published on the activity topic. Like reported(), it may be
  /// called from another thread at any time.
  virtual ReportedState activity() = 0;

  /// Bring everything down as on process exit. Cycles may still run while it executes, and
  /// cycle() is never called after it returned. A component whose call still hangs may be finished
  /// once that call returned.
  virtual void shutdown() = 0;
};

using CmFactory = std::function<std::unique_ptr<CmUnderTest>(const CmConfig &)>;

}  // namespace cm_test_suite

#endif  // CM_TEST_SUITE__CM_UNDER_TEST_HPP_
