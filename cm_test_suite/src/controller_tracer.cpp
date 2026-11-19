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

#include "controller_tracer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <tuple>

namespace cm_test_suite
{
namespace
{
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using controller_interface::return_type;

uint16_t required_controller_index(std::string_view name)
{
  const auto index = rig::controller_index(name);
  if (!index)
  {
    throw std::invalid_argument("Controller " + std::string(name) + " is not part of the rig");
  }
  return *index;
}

uint16_t required_interface_id(const std::string & name)
{
  const auto id = rig::interface_id(name);
  if (!id)
  {
    throw std::logic_error("Rig interface " + name + " has no id");
  }
  return *id;
}

std::vector<uint16_t> interface_ids(
  const std::string & prefix, const std::vector<std::string> & names)
{
  std::vector<uint16_t> ids;
  for (const auto & name : names)
  {
    ids.push_back(required_interface_id(prefix.empty() ? name : prefix + "/" + name));
  }
  return ids;
}

std::vector<std::pair<size_t, uint16_t>> exported_inputs(const rig::Controller & controller)
{
  std::vector<std::pair<size_t, uint16_t>> inputs;
  for (size_t i = 0; i < controller.state_interfaces.size(); ++i)
  {
    const auto & name = controller.state_interfaces[i];
    if (!rig::hardware_of_interface(name))
    {
      inputs.emplace_back(i, required_interface_id(name));
    }
  }
  return inputs;
}

std::vector<uint16_t> controller_indices(const rig::Names & names)
{
  std::vector<uint16_t> indices;
  for (const auto & name : names)
  {
    indices.push_back(required_controller_index(name));
  }
  return indices;
}

std::vector<uint16_t> hardware_indices(const rig::Names & names)
{
  std::vector<uint16_t> indices;
  for (const auto & name : names)
  {
    indices.push_back(*rig::hardware_index(name));
  }
  return indices;
}

InterfaceConfiguration individual(const std::vector<std::string> & names, bool best_effort)
{
  if (names.empty())
  {
    return {interface_configuration_type::NONE, {}};
  }
  return {
    best_effort ? interface_configuration_type::INDIVIDUAL_BEST_EFFORT
                : interface_configuration_type::INDIVIDUAL,
    names};
}

/// The interfaces a best effort claim may get, those that exist on readable hardware, and those it
/// must get. Faulted hardware is still in its fault handling, such as a slow on_error, and the
/// controller manager may give up on hanging hardware like on a fault, so it may already count
/// their states as unavailable.
std::pair<std::vector<std::string>, std::vector<std::string>> available_states(
  const rig::Controller & controller)
{
  std::vector<std::string> allowed;
  std::vector<std::string> required;
  for (const auto & name : controller.state_interfaces)
  {
    if (!rig::hardware_state_exists(name))
    {
      continue;
    }
    const uint16_t index = *rig::hardware_index(*rig::hardware_of_interface(name));
    const auto hardware = trace().hardware(index);
    if (hardware.state == Lifecycle::Active || hardware.state == Lifecycle::Inactive)
    {
      allowed.push_back(name);
      if (!hardware.faulted && !trace().hardware_hanging(index))
      {
        required.push_back(name);
      }
    }
  }
  return {allowed, required};
}

template <typename Loaned>
bool names_equal(const std::vector<Loaned> & loaned, const std::vector<std::string> & expected)
{
  if (loaned.size() != expected.size())
  {
    return false;
  }
  for (size_t i = 0; i < loaned.size(); ++i)
  {
    if (loaned[i].get_name() != expected[i])
    {
      return false;
    }
  }
  return true;
}

/// The loaned names are the allowed ones in order, without some that are not required.
template <typename Loaned>
bool names_within(
  const std::vector<Loaned> & loaned, const std::vector<std::string> & allowed,
  const std::vector<std::string> & required)
{
  size_t next = 0;
  for (const auto & interface : loaned)
  {
    while (next < allowed.size() && allowed[next] != interface.get_name())
    {
      if (std::find(required.begin(), required.end(), allowed[next]) != required.end())
      {
        return false;
      }
      ++next;
    }
    if (next == allowed.size())
    {
      return false;
    }
    ++next;
  }
  for (; next < allowed.size(); ++next)
  {
    if (std::find(required.begin(), required.end(), allowed[next]) != required.end())
    {
      return false;
    }
  }
  return true;
}

std::vector<bool> probed_commands(const rig::Controller & controller)
{
  std::vector<bool> probed;
  for (const auto & name : controller.command_interfaces)
  {
    const auto hardware = rig::hardware_of_interface(name);
    probed.push_back(
      hardware && rig::limit_probed(
                    rig::hardware()[*rig::hardware_index(*hardware)],
                    std::string_view(name).substr(name.find('/') + 1)));
  }
  return probed;
}
}  // namespace

ControllerTracer::ControllerTracer(std::string_view name)
: index_(required_controller_index(name)),
  definition_(rig::controllers()[index_]),
  target_{TargetKind::Controller, index_},
  command_ids_(interface_ids("", definition_.command_interfaces)),
  probed_commands_(probed_commands(definition_)),
  reference_ids_(interface_ids(definition_.name, definition_.references)),
  exported_state_ids_(interface_ids(definition_.name, definition_.exported_states)),
  exported_inputs_(exported_inputs(definition_)),
  providers_(controller_indices(rig::providers(definition_.name))),
  reference_providers_(controller_indices(rig::written_providers(definition_.name))),
  users_(controller_indices(rig::users(definition_.name))),
  conflicts_(controller_indices(rig::command_conflicts(definition_.name))),
  commanded_hardware_(hardware_indices(rig::commanded_hardware_of(definition_.name))),
  read_hardware_(hardware_indices(rig::hardware_of(definition_.name))),
  expected_states_(definition_.state_interfaces),
  required_states_(definition_.state_interfaces)
{
  if (definition_.best_effort_states && !exported_inputs_.empty())
  {
    throw std::logic_error("Best effort rig controller " + definition_.name + " reads exports");
  }
  if (definition_.chainable && definition_.references.empty())
  {
    throw std::logic_error("Chainable rig controller " + definition_.name + " has no references");
  }
  trace().set_controller_chained(index_, false);
  trace().set_controller_state(index_, Lifecycle::Unconfigured);
}

ControllerTracer::~ControllerTracer() { trace().set_controller_state(index_, Lifecycle::Unloaded); }

InterfaceConfiguration ControllerTracer::command_configuration() const
{
  return individual(definition_.command_interfaces, false);
}

InterfaceConfiguration ControllerTracer::state_configuration() const
{
  return individual(definition_.state_interfaces, definition_.best_effort_states);
}

CallbackReturn ControllerTracer::configure()
{
  auto busy = guard();
  const PendingCall pending;
  if (auto forced = take_lifecycle_fault(target_, Hook::Configure, busy))
  {
    return *forced;
  }
  trace().set_controller_state(index_, Lifecycle::Inactive);
  return CallbackReturn::SUCCESS;
}

CallbackReturn ControllerTracer::activate(
  const CommandInterfaces & commands, const StateInterfaces & states)
{
  auto busy = guard();
  const PendingCall pending;
  report_if_inactive(providers_, EventKind::ActivatedBeforeProvider);
  for (const auto provider : reference_providers_)
  {
    if (!trace().controller(provider).chained)
    {
      report(EventKind::ActivatedOnUnchainedProvider, NO_INDEX, provider);
    }
  }
  for (const auto hardware : read_hardware_)
  {
    const auto state = trace().hardware(hardware).state;
    const bool commanded =
      std::find(commanded_hardware_.begin(), commanded_hardware_.end(), hardware) !=
      commanded_hardware_.end();
    const bool may_skip = definition_.best_effort_states && !commanded;
    if (state != Lifecycle::Active && (commanded || state != Lifecycle::Inactive) && !may_skip)
    {
      report(EventKind::ActivatedOnInactiveHardware);
    }
  }
  for (const auto other : conflicts_)
  {
    // A hanging controller is out of the running set, a failed one may still write in on_deactivate
    if (trace().controller(other).state == Lifecycle::Active && !trace().controller_hanging(other))
    {
      report(EventKind::SharedCommandInterface, NO_INDEX, other);
    }
  }
  if (definition_.best_effort_states)
  {
    std::tie(expected_states_, required_states_) = available_states(definition_);
  }
  if (!interfaces_match(commands, states))
  {
    report(EventKind::MissingInterfaces);
  }
  const Fault fault = faults().take(target_, Hook::Activate);
  quiet_updates_ = fault == Fault::Quiet ? QUIET_UPDATES : 0;
  if (auto forced = apply_lifecycle_fault(fault, target_, Hook::Activate, busy))
  {
    return *forced;
  }
  trace().set_controller_state(index_, Lifecycle::Active);
  return CallbackReturn::SUCCESS;
}

CallbackReturn ControllerTracer::deactivate()
{
  auto busy = guard();
  const PendingCall pending;
  for (const auto user : users_)
  {
    // A user hanging in a callback is out of the running set (rulebook 2.2.5)
    if (trace().controller(user).state == Lifecycle::Active && !trace().controller_hanging(user))
    {
      report(EventKind::ProviderStoppedUnderUser, NO_INDEX, user);
    }
  }
  if (auto forced = take_lifecycle_fault(target_, Hook::Deactivate, busy))
  {
    if (*forced == CallbackReturn::FAILURE)
    {
      trace().set_controller_refused_stop(index_);
    }
    return *forced;
  }
  trace().set_controller_state(index_, Lifecycle::Inactive);
  return CallbackReturn::SUCCESS;
}

CallbackReturn ControllerTracer::cleanup()
{
  auto busy = guard();
  const PendingCall pending;
  if (auto forced = take_lifecycle_fault(target_, Hook::Cleanup, busy))
  {
    return *forced;
  }
  trace().set_controller_state(index_, Lifecycle::Unconfigured);
  return CallbackReturn::SUCCESS;
}

CallbackReturn ControllerTracer::error()
{
  auto busy = guard();
  const PendingCall pending;
  // A failing or throwing on_error ends in finalized
  std::optional<CallbackReturn> forced;
  try
  {
    forced = take_lifecycle_fault(target_, Hook::Error, busy);
  }
  catch (const InjectedFault &)
  {
    trace().set_controller_state(index_, Lifecycle::Finalized);
    throw;
  }
  if (forced)
  {
    trace().set_controller_state(index_, Lifecycle::Finalized);
    return *forced;
  }
  trace().set_controller_state(index_, Lifecycle::Unconfigured);
  return CallbackReturn::SUCCESS;
}

CallbackReturn ControllerTracer::shutdown()
{
  auto busy = guard();
  const PendingCall pending;
  const auto forced = take_lifecycle_fault(target_, Hook::Shutdown, busy);
  trace().set_controller_state(index_, Lifecycle::Finalized);
  return forced.value_or(CallbackReturn::SUCCESS);
}

bool ControllerTracer::set_chained_mode(bool chained)
{
  auto busy = guard();
  const PendingCall pending;
  if (faults().take(target_, Hook::ChainedMode) == Fault::Failure)
  {
    return false;
  }
  trace().set_controller_chained(index_, chained);
  return true;
}

std::optional<return_type> ControllerTracer::begin_update(
  const CommandInterfaces & commands, const StateInterfaces & states)
{
  trace().count_update(index_);
  const auto truth = trace().controller(index_);
  if (truth.state != Lifecycle::Active)
  {
    report(EventKind::UpdateWhileInactive);
  }
  if (truth.failed_epoch != 0 && truth.failed_epoch == truth.epoch)
  {
    report(EventKind::UpdateAfterFailure);
  }
  if (!interfaces_match(commands, states))
  {
    report(EventKind::MissingInterfaces);
    trace().set_controller_failed(index_);
    return return_type::ERROR;
  }
  switch (faults().take(target_, Hook::Update))
  {
    case Fault::Error:
      trace().set_controller_failed(index_);
      return return_type::ERROR;
    case Fault::Throw:
      trace().set_controller_failed(index_);
      throw InjectedFault(Hook::Update);
    default:
      break;
  }
  for (const auto & [state_index, interface] : exported_inputs_)
  {
    const auto value = states[state_index].get_optional<double>();
    if (!value || std::isnan(*value))
    {
      continue;
    }
    if (auto event = trace().audit(decode_token(*value), interface, target_))
    {
      trace().report(*event);
    }
  }
  return std::nullopt;
}

bool ControllerTracer::quiet_update()
{
  const int left = quiet_updates_.load();
  if (left <= 0)
  {
    return false;
  }
  quiet_updates_ = left - 1;
  return true;
}

double ControllerTracer::mint_command(size_t index, uint64_t parent)
{
  const uint64_t token = trace().mint(index_, command_ids_.at(index), parent);
  const bool probe = probed_commands_.at(index) && token % rig::LIMIT_PROBE_PERIOD == 0;
  return probe ? rig::LIMIT_PROBE : encode_token(token);
}

double ControllerTracer::mint_reference(size_t index)
{
  return encode_token(trace().mint(index_, reference_ids_.at(index), 0));
}

double ControllerTracer::mint_exported_state(size_t index)
{
  return encode_token(trace().mint(index_, exported_state_ids_.at(index), 0));
}

std::optional<uint64_t> ControllerTracer::audit_reference(size_t index, std::optional<double> value)
{
  if (!value || std::isnan(*value))
  {
    return std::nullopt;
  }
  const uint64_t token = decode_token(*value);
  if (auto event = trace().audit(token, reference_ids_.at(index), target_))
  {
    trace().report(*event);
  }
  if (token == 0)
  {
    return std::nullopt;
  }
  return token;
}

bool ControllerTracer::interfaces_match(
  const CommandInterfaces & commands, const StateInterfaces & states) const
{
  return names_equal(commands, definition_.command_interfaces) &&
         names_within(states, expected_states_, required_states_);
}

void ControllerTracer::report(EventKind kind, uint16_t interface, uint16_t writer) const
{
  trace().report({kind, target_, interface, writer, trace().cycle()});
}

void ControllerTracer::report_if_inactive(
  const std::vector<uint16_t> & controllers, EventKind kind) const
{
  for (const auto controller : controllers)
  {
    if (trace().controller(controller).state != Lifecycle::Active)
    {
      report(kind, NO_INDEX, controller);
    }
  }
}

}  // namespace cm_test_suite
