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

#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "cm_test_suite/suite.hpp"
#include "controller_manager/controller_manager.hpp"
#include "controller_manager_msgs/msg/controller_manager_activity.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/list_hardware_components.hpp"
#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"

namespace
{
using cm_test_suite::Lifecycle;
using controller_interface::return_type;
using SwitchRequest = controller_manager_msgs::srv::SwitchController::Request;

/// Exposes the service callbacks, so reported() returns exactly what the services return.
class ExposedControllerManager : public controller_manager::ControllerManager
{
public:
  using controller_manager::ControllerManager::ControllerManager;
  using controller_manager::ControllerManager::list_controllers_srv_cb;
  using controller_manager::ControllerManager::list_hardware_components_srv_cb;
  using controller_manager::ControllerManager::set_hardware_component_state_srv_cb;

  bool shutdown_components() { return resource_manager_->shutdown_components(); }
};

/// Primary lifecycle states. Transition states such as deactivating have none.
std::optional<Lifecycle> lifecycle_from_label(const std::string & label)
{
  static const std::map<std::string, Lifecycle> primary = {
    {"unconfigured", Lifecycle::Unconfigured},
    {"inactive", Lifecycle::Inactive},
    {"active", Lifecycle::Active},
    {"finalized", Lifecycle::Finalized}};
  static const std::set<std::string> transitions = {
    "configuring", "cleaningup", "activating", "deactivating", "shuttingdown", "errorprocessing"};
  const auto it = primary.find(label);
  if (it != primary.end())
  {
    return it->second;
  }
  if (transitions.count(label) == 0)
  {
    throw std::runtime_error("Unknown lifecycle label '" + label + "'");
  }
  return std::nullopt;
}

/// Adds one reported entry, or marks it as in transition.
void add_entry(
  std::map<std::string, Lifecycle> & entries, cm_test_suite::ReportedState & state,
  const std::string & name, const std::string & label)
{
  if (const auto lifecycle = lifecycle_from_label(label))
  {
    entries[name] = *lifecycle;
  }
  else
  {
    state.transitioning.insert(name);
  }
}

int strictness_of(cm_test_suite::Strictness strictness)
{
  switch (strictness)
  {
    case cm_test_suite::Strictness::Strict:
      return SwitchRequest::STRICT;
    case cm_test_suite::Strictness::Auto:
      return SwitchRequest::AUTO;
    case cm_test_suite::Strictness::ForceAuto:
      return SwitchRequest::FORCE_AUTO;
  }
  throw std::logic_error("Unknown strictness");
}

/// The ros2_control controller manager as it is in this workspace.
class LegacyCm : public cm_test_suite::CmUnderTest
{
public:
  LegacyCm(const cm_test_suite::CmConfig & config, bool activate_asap)
  : executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>()),
    cm_(
      std::make_shared<ExposedControllerManager>(
        executor_, config.robot_description, true, "cm_under_test")),
    listener_(std::make_shared<rclcpp::Node>("cm_under_test_listener")),
    time_(0, 0, cm_->get_trigger_clock()->get_clock_type()),
    switch_timeout_(rclcpp::Duration(config.switch_timeout)),
    activate_asap_(activate_asap)
  {
    activity_subscription_ =
      listener_->create_subscription<controller_manager_msgs::msg::ControllerManagerActivity>(
        "/cm_under_test/activity", rclcpp::QoS(1).reliable().transient_local(),
        [this](const controller_manager_msgs::msg::ControllerManagerActivity & message)
        {
          cm_test_suite::ReportedState state;
          for (const auto & named : message.controllers)
          {
            add_entry(state.controllers, state, named.name, named.state.label);
          }
          for (const auto & named : message.hardware_components)
          {
            add_entry(state.hardware, state, named.name, named.state.label);
          }
          std::lock_guard<std::mutex> lock(activity_mutex_);
          activity_ = std::move(state);
        });
    // Like ros2_control_node, which spins the controller manager next to the control loop
    executor_->add_node(cm_);
    executor_->add_node(listener_);
    spinner_ = std::thread([this] { executor_->spin(); });
  }

  ~LegacyCm() override
  {
    executor_->cancel();
    spinner_.join();
  }

  void cycle() override
  {
    cm_->read(time_, PERIOD);
    cm_->update(time_, PERIOD);
    cm_->write(time_, PERIOD);
    time_ += PERIOD;
  }

  bool load(const std::string & controller) override
  {
    const auto & definition = cm_test_suite::rig::controller(controller);
    return cm_->load_controller(controller, cm_test_suite::rig::plugin_type(definition)) != nullptr;
  }

  bool configure(const std::string & controller) override
  {
    return cm_->configure_controller(controller) == return_type::OK;
  }

  bool cleanup(const std::string & controller) override
  {
    return cm_->cleanup_controller(controller) == return_type::OK;
  }

  bool unload(const std::string & controller) override
  {
    return cm_->unload_controller(controller) == return_type::OK;
  }

  bool switch_controllers(
    const std::vector<std::string> & activate, const std::vector<std::string> & deactivate,
    cm_test_suite::Strictness strictness) override
  {
    return cm_->switch_controller(
             activate, deactivate, strictness_of(strictness), activate_asap_, switch_timeout_) ==
           return_type::OK;
  }

  bool recover_hardware(const std::string & hardware) override
  {
    auto request =
      std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>();
    request->name = hardware;
    request->target_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    request->target_state.label = "active";
    auto response =
      std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Response>();
    cm_->set_hardware_component_state_srv_cb(request, response);
    return response->ok;
  }

  cm_test_suite::ReportedState reported() override
  {
    cm_test_suite::ReportedState state;
    auto controllers = std::make_shared<controller_manager_msgs::srv::ListControllers::Response>();
    cm_->list_controllers_srv_cb(
      std::make_shared<controller_manager_msgs::srv::ListControllers::Request>(), controllers);
    for (const auto & controller : controllers->controller)
    {
      add_entry(state.controllers, state, controller.name, controller.state);
    }
    auto hardware =
      std::make_shared<controller_manager_msgs::srv::ListHardwareComponents::Response>();
    cm_->list_hardware_components_srv_cb(
      std::make_shared<controller_manager_msgs::srv::ListHardwareComponents::Request>(), hardware);
    for (const auto & component : hardware->component)
    {
      add_entry(state.hardware, state, component.name, component.state.label);
    }
    return state;
  }

  cm_test_suite::ReportedState activity() override
  {
    std::lock_guard<std::mutex> lock(activity_mutex_);
    return activity_;
  }

  void shutdown() override
  {
    // What the controller manager's pre-shutdown callback does
    cm_->shutdown_controllers();
    cm_->shutdown_components();
  }

private:
  static inline const rclcpp::Duration PERIOD = rclcpp::Duration::from_seconds(0.01);

  std::shared_ptr<rclcpp::Executor> executor_;
  std::shared_ptr<ExposedControllerManager> cm_;
  rclcpp::Node::SharedPtr listener_;
  rclcpp::Subscription<controller_manager_msgs::msg::ControllerManagerActivity>::SharedPtr
    activity_subscription_;
  std::mutex activity_mutex_;
  cm_test_suite::ReportedState activity_;
  std::thread spinner_;
  rclcpp::Time time_;
  const rclcpp::Duration switch_timeout_;
  const bool activate_asap_;
};
}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  if (!std::getenv("CM_FUZZ_VERBOSE"))
  {
    rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_FATAL);
  }
  // Upstream answers AUTO and FORCE_AUTO with BEST_EFFORT, so it has neither
  cm_test_suite::Capabilities capabilities;
  capabilities.recover_hardware = true;
  for (const bool asap : {false, true})
  {
    cm_test_suite::register_suite(
      asap ? "LegacyAsap" : "Legacy", capabilities, [asap](const cm_test_suite::CmConfig & config)
      { return std::make_unique<LegacyCm>(config, asap); });
  }
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
