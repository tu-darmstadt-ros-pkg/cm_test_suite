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

#ifndef CM_TEST_SUITE__RIG_HPP_
#define CM_TEST_SUITE__RIG_HPP_

#include <cstdint>
#include <functional>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <vector>

/// The one fixed robot every controller manager under test runs.
/// Plugins look up their configuration here by name, so no parameters are needed.
namespace cm_test_suite::rig
{
using Names = std::set<std::string>;

struct Hardware
{
  std::string name;
  bool is_async;
  /// Whether read and write fail without power, returning ERROR from the POWER_LOSS_ERROR_AFTER th
  /// call on.
  bool faults_on_power_loss;
  std::vector<std::string> joints;
  /// Command interfaces of every joint. A joint accepts only one active mode at a time.
  std::vector<std::string> command_modes;
  std::vector<std::string> joint_states;
  /// Full names of sensor state interfaces, the prefix is the sensor name.
  std::vector<std::string> sensor_states;
};

struct Controller
{
  std::string name;
  bool chainable;
  std::vector<std::string> command_interfaces;
  std::vector<std::string> state_interfaces;
  std::vector<std::string> references;
  std::vector<std::string> exported_states;
  /// Claims its state interfaces with INDIVIDUAL_BEST_EFFORT, like joint_state_broadcaster.
  bool best_effort_states = false;
};

struct StartupProfile
{
  Names hardware_active;
  Names controllers_loaded;
  Names controllers_active;
};

/// Like a bus driver that tolerates a few failed transfers before it reports an error.
constexpr int POWER_LOSS_ERROR_AFTER = 3;

/// Joint limits. Joints with only a velocity mode are continuous without position limits. Tokens
/// lie below 2^53 and change by less than a velocity limited step at any update rate, so a
/// controller manager that enforces the limits leaves them unchanged.
constexpr double POSITION_LIMIT = 9007199254740992.0;
constexpr double VELOCITY_LIMIT = 1e30;
/// What a controller writes into a hardware command instead of every LIMIT_PROBE_PERIOD th token.
/// It lies beyond every limit, so the hardware may only apply it clamped onto one.
constexpr double LIMIT_PROBE = 1e31;
constexpr uint64_t LIMIT_PROBE_PERIOD = 16;

constexpr char TRACE_CONTROLLER[] = "cm_test_suite/TraceController";
constexpr char TRACE_CHAINABLE_CONTROLLER[] = "cm_test_suite/TraceChainableController";
constexpr char TRACE_SYSTEM[] = "cm_test_suite/TraceSystem";

/// With CM_FUZZ_ALL_ASYNC=1 every component is async, so arm runs like Athena's arm in velocity
/// and flipper like its flippers in position, without a watchdog.
const std::vector<Hardware> & hardware();
const std::vector<Controller> & controllers();
const StartupProfile & startup_profile();

std::optional<uint16_t> hardware_index(std::string_view name);
std::optional<uint16_t> controller_index(std::string_view name);
const Controller & controller(std::string_view name);
const char * plugin_type(const Controller & controller);

std::string urdf();
/// The rig description with flipper also exporting the interfaces of joint a1, which arm exports.
std::string urdf_with_duplicate_interfaces();

/// Ids of every command, reference and exported state interface. Tokens carry them.
std::optional<uint16_t> interface_id(std::string_view full_name);
const std::string & interface_name(uint16_t id);
size_t interface_count();

/// Whether a hardware state interface of that name exists in the rig.
bool hardware_state_exists(std::string_view full_name);

/// Hardware owning a hardware interface, empty for interfaces exported by controllers.
std::optional<std::string> hardware_of_interface(std::string_view full_name);

/// Whether controllers probe the limits in this command mode of the hardware. A limiter may
/// predict an async joint from its last velocity command, so a clamped probe would take the
/// predicted joint onto its position limit and hold the next commands there.
bool limit_probed(const Hardware & hw, std::string_view mode);

/// Hardware whose interfaces the controller claims itself.
Names hardware_of(std::string_view controller);
/// Hardware whose command interfaces the controller claims. It must be active, while hardware
/// that is only read may also be inactive.
Names commanded_hardware_of(std::string_view controller);
/// Controllers whose references or exported states the controller claims.
Names providers(std::string_view controller);
/// Controllers whose references the controller claims, the inverse of reference_writers.
Names written_providers(std::string_view controller);
/// Controllers claiming references or exported states of the controller.
Names users(std::string_view controller);
/// Controllers claiming references of the controller. Its chained mode follows them.
Names reference_writers(std::string_view controller);
/// Controllers claiming a command interface the controller also claims.
Names claim_conflicts(std::string_view controller);
/// The claim conflicts and the controllers commanding another mode of a joint the controller
/// commands. None of them may be active together with it.
Names command_conflicts(std::string_view controller);

/// Transitive closure of a relation, without the start set unless reached again.
Names closure(const Names & start, const std::function<Names(std::string_view)> & relation);
/// Controllers connected to the start set by any chain edge, including the start set.
Names connected(const Names & start);

}  // namespace cm_test_suite::rig

#endif  // CM_TEST_SUITE__RIG_HPP_
