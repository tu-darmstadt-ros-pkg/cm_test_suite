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

#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_tracer.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace cm_test_suite
{
namespace
{
using controller_interface::InterfaceConfiguration;
using controller_interface::return_type;

constexpr double NO_COMMAND = std::numeric_limits<double>::quiet_NaN();

CallbackReturn init_tracer(
  std::optional<ControllerTracer> & tracer, const rclcpp_lifecycle::LifecycleNode & node)
{
  try
  {
    tracer.emplace(node.get_name());
    return CallbackReturn::SUCCESS;
  }
  catch (const std::invalid_argument & e)
  {
    RCLCPP_ERROR(node.get_logger(), "%s", e.what());
    return CallbackReturn::ERROR;
  }
}

template <typename Interface>
void write(Interface & interface, double value)
{
  // A write lost to a busy handle leaves the previous token, which the audits report as stale
  std::ignore = interface.set_value(value);
}
}  // namespace

/// Rig controller without references. Writes a fresh token into every command each update.
class TraceController : public controller_interface::ControllerInterface
{
public:
  CallbackReturn on_init() override { return init_tracer(tracer_, *get_node()); }

  InterfaceConfiguration command_interface_configuration() const override
  {
    return tracer_->command_configuration();
  }

  InterfaceConfiguration state_interface_configuration() const override
  {
    return tracer_->state_configuration();
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    return tracer_->configure();
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    return tracer_->activate(command_interfaces_, state_interfaces_);
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    return tracer_->deactivate();
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override { return tracer_->cleanup(); }

  CallbackReturn on_error(const rclcpp_lifecycle::State &) override { return tracer_->error(); }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    return tracer_->shutdown();
  }

  return_type update(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    auto busy = tracer_->guard();
    if (auto result = tracer_->begin_update(command_interfaces_, state_interfaces_))
    {
      return *result;
    }
    if (tracer_->quiet_update())
    {
      return return_type::OK;
    }
    for (size_t i = 0; i < command_interfaces_.size(); ++i)
    {
      write(command_interfaces_[i], tracer_->mint_command(i, 0));
    }
    return return_type::OK;
  }

private:
  std::optional<ControllerTracer> tracer_;
};

/// Rig controller with references. In chained mode every command derives from the token in its
/// reference, so the hardware can trace a command back through the whole chain.
class TraceChainableController : public controller_interface::ChainableControllerInterface
{
public:
  CallbackReturn on_init() override { return init_tracer(tracer_, *get_node()); }

  InterfaceConfiguration command_interface_configuration() const override
  {
    return tracer_->command_configuration();
  }

  InterfaceConfiguration state_interface_configuration() const override
  {
    return tracer_->state_configuration();
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    return tracer_->configure();
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    reset_references();
    return tracer_->activate(command_interfaces_, state_interfaces_);
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    return tracer_->deactivate();
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override { return tracer_->cleanup(); }

  CallbackReturn on_error(const rclcpp_lifecycle::State &) override { return tracer_->error(); }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    return tracer_->shutdown();
  }

protected:
  std::vector<hardware_interface::StateInterface::SharedPtr> on_export_state_interfaces_list()
    override
  {
    std::vector<hardware_interface::StateInterface::SharedPtr> interfaces;
    for (const auto & name : tracer_->definition().exported_states)
    {
      interfaces.push_back(
        std::make_shared<hardware_interface::StateInterface>(get_node()->get_name(), name));
      write(*interfaces.back(), NO_COMMAND);
    }
    return interfaces;
  }

  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_reference_interfaces_list()
    override
  {
    std::vector<hardware_interface::CommandInterface::SharedPtr> interfaces;
    for (const auto & name : tracer_->definition().references)
    {
      interfaces.push_back(
        std::make_shared<hardware_interface::CommandInterface>(get_node()->get_name(), name));
      write(*interfaces.back(), NO_COMMAND);
    }
    return interfaces;
  }

  // Like Hector's controllers, references are reset in both directions
  bool on_set_chained_mode(bool chained_mode) override
  {
    if (!tracer_->set_chained_mode(chained_mode))
    {
      return false;
    }
    reset_references();
    return true;
  }

  return_type update_reference_from_subscribers(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    auto busy = tracer_->guard();
    for (size_t i = 0; i < ordered_exported_reference_interfaces_.size(); ++i)
    {
      write(*ordered_exported_reference_interfaces_[i], tracer_->mint_reference(i));
    }
    return return_type::OK;
  }

  return_type update_and_write_commands(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    auto busy = tracer_->guard();
    if (auto result = tracer_->begin_update(command_interfaces_, state_interfaces_))
    {
      return *result;
    }
    const auto & references = ordered_exported_reference_interfaces_;
    const size_t commands = tracer_->quiet_update() ? 0 : command_interfaces_.size();
    for (size_t i = 0; i < commands; ++i)
    {
      const size_t reference = i % references.size();
      const auto parent =
        tracer_->audit_reference(reference, references[reference]->get_optional<double>());
      write(command_interfaces_[i], parent ? tracer_->mint_command(i, *parent) : NO_COMMAND);
    }
    for (size_t i = 0; i < ordered_exported_state_interfaces_.size(); ++i)
    {
      write(*ordered_exported_state_interfaces_[i], tracer_->mint_exported_state(i));
    }
    return return_type::OK;
  }

private:
  void reset_references()
  {
    for (const auto & reference : ordered_exported_reference_interfaces_)
    {
      write(*reference, NO_COMMAND);
    }
  }

  std::optional<ControllerTracer> tracer_;
};

}  // namespace cm_test_suite

// cppcheck-suppress unknownMacro
PLUGINLIB_EXPORT_CLASS(cm_test_suite::TraceController, controller_interface::ControllerInterface)
PLUGINLIB_EXPORT_CLASS(
  cm_test_suite::TraceChainableController, controller_interface::ChainableControllerInterface)
