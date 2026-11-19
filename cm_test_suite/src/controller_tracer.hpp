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

#ifndef CONTROLLER_TRACER_HPP_
#define CONTROLLER_TRACER_HPP_

#include <atomic>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

#include "cm_test_suite/rig.hpp"
#include "controller_interface/controller_interface_base.hpp"
#include "plugin_support.hpp"

namespace cm_test_suite
{
/// Behaviour shared by the plain and the chainable rig controller.
///
/// It records the controller's lifecycle in the trace, checks the chain and hardware state the
/// controller manager must guarantee at each callback, and mints the tokens the controller writes.
class ControllerTracer
{
public:
  using CommandInterfaces = std::vector<hardware_interface::LoanedCommandInterface>;
  using StateInterfaces = std::vector<hardware_interface::LoanedStateInterface>;

  /// Throws std::invalid_argument for a name that is not in the rig.
  explicit ControllerTracer(std::string_view name);
  ~ControllerTracer();

  ControllerTracer(const ControllerTracer &) = delete;
  ControllerTracer & operator=(const ControllerTracer &) = delete;

  const rig::Controller & definition() const { return definition_; }
  controller_interface::InterfaceConfiguration command_configuration() const;
  controller_interface::InterfaceConfiguration state_configuration() const;

  CallGuard guard() { return CallGuard(busy_, EventKind::ConcurrentControllerCall, target_); }

  CallbackReturn configure();
  CallbackReturn activate(const CommandInterfaces & commands, const StateInterfaces & states);
  CallbackReturn deactivate();
  CallbackReturn cleanup();
  CallbackReturn error();
  CallbackReturn shutdown();
  bool set_chained_mode(bool chained);

  /// Returns the result the update must return right away, if it must not write.
  std::optional<controller_interface::return_type> begin_update(
    const CommandInterfaces & commands, const StateInterfaces & states);

  /// Whether this update writes no command, in the first updates after a quiet activation.
  bool quiet_update();

  /// A token, or now and then LIMIT_PROBE for a hardware command.
  double mint_command(size_t index, uint64_t parent);
  double mint_reference(size_t index);
  double mint_exported_state(size_t index);

  /// The token a command derives from the reference value, or nullopt for no command.
  std::optional<uint64_t> audit_reference(size_t index, std::optional<double> value);

private:
  bool interfaces_match(const CommandInterfaces & commands, const StateInterfaces & states) const;
  void report(EventKind kind, uint16_t interface = NO_INDEX, uint16_t writer = NO_INDEX) const;
  void report_if_inactive(const std::vector<uint16_t> & controllers, EventKind kind) const;

  const uint16_t index_;
  const rig::Controller & definition_;
  const Target target_;
  const std::vector<uint16_t> command_ids_;
  /// Whether each command interface belongs to hardware whose limits a probe tests.
  const std::vector<bool> probed_commands_;
  const std::vector<uint16_t> reference_ids_;
  const std::vector<uint16_t> exported_state_ids_;
  /// State interfaces exported by other controllers, as (state index, interface id).
  const std::vector<std::pair<size_t, uint16_t>> exported_inputs_;
  const std::vector<uint16_t> providers_;
  const std::vector<uint16_t> reference_providers_;
  const std::vector<uint16_t> users_;
  /// Controllers that may not be active together with it.
  const std::vector<uint16_t> conflicts_;
  const std::vector<uint16_t> commanded_hardware_;
  /// All hardware the controller uses. Read only hardware may be inactive.
  const std::vector<uint16_t> read_hardware_;
  /// The state interfaces it may hold while active, in order, and those it must hold. Best effort
  /// claims depend on the hardware.
  std::vector<std::string> expected_states_;
  std::vector<std::string> required_states_;
  std::atomic<bool> busy_{false};
  std::atomic<int> quiet_updates_{0};
};

}  // namespace cm_test_suite

#endif  // CONTROLLER_TRACER_HPP_
