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

#include "cm_test_suite/rig.hpp"

#include <algorithm>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace cm_test_suite::rig
{
namespace
{
std::string_view prefix(std::string_view full_name)
{
  return full_name.substr(0, full_name.find('/'));
}

Names joints_commanded_by(std::string_view controller_name)
{
  Names joints;
  for (const auto & interface : controller(controller_name).command_interfaces)
  {
    if (hardware_of_interface(interface))
    {
      joints.insert(std::string(prefix(interface)));
    }
  }
  return joints;
}

bool claims_any(const std::vector<std::string> & interfaces, std::string_view controller)
{
  return std::any_of(
    interfaces.begin(), interfaces.end(),
    [&](const std::string & name) { return prefix(name) == controller; });
}

std::vector<std::string> build_interface_table()
{
  std::vector<std::string> table;
  for (const auto & hw : hardware())
  {
    for (const auto & joint : hw.joints)
    {
      for (const auto & mode : hw.command_modes)
      {
        table.push_back(joint + "/" + mode);
      }
    }
  }
  for (const auto & c : controllers())
  {
    for (const auto & reference : c.references)
    {
      table.push_back(c.name + "/" + reference);
    }
    for (const auto & state : c.exported_states)
    {
      table.push_back(c.name + "/" + state);
    }
  }
  return table;
}

const std::vector<std::string> & interface_table()
{
  static const std::vector<std::string> table = build_interface_table();
  return table;
}

template <typename T>
std::optional<uint16_t> index_by_name(const std::vector<T> & items, std::string_view name)
{
  for (size_t i = 0; i < items.size(); ++i)
  {
    if (items[i].name == name)
    {
      return static_cast<uint16_t>(i);
    }
  }
  return std::nullopt;
}

std::string number(double value)
{
  std::ostringstream text;
  text << std::setprecision(17) << value;
  return text.str();
}

/// A joint with only a velocity mode turns without end like a wheel.
bool continuous(const Hardware & hw)
{
  return hw.command_modes == std::vector<std::string>{"velocity"};
}

std::string hardware_xml(const Hardware & hw)
{
  std::string xml = "<ros2_control name=\"" + hw.name + "\" type=\"system\" is_async=\"" +
                    (hw.is_async ? "true" : "false") + "\"><hardware><plugin>" + TRACE_SYSTEM +
                    "</plugin></hardware>";
  for (const auto & joint : hw.joints)
  {
    xml += "<joint name=\"" + joint + "\">";
    for (const auto & mode : hw.command_modes)
    {
      xml += "<command_interface name=\"" + mode + "\"/>";
    }
    // A limited position command starts from the actual position, which must be known
    for (const auto & state : hw.joint_states)
    {
      xml += "<state_interface name=\"" + state +
             "\"><param name=\"initial_value\">0.0</param></state_interface>";
    }
    xml += "</joint>";
  }
  for (const auto & state : hw.sensor_states)
  {
    const auto slash = state.find('/');
    xml += "<sensor name=\"" + state.substr(0, slash) + "\"><state_interface name=\"" +
           state.substr(slash + 1) + "\"/></sensor>";
  }
  return xml + "</ros2_control>";
}
}  // namespace

const std::vector<Hardware> & hardware()
{
  // flipper keeps running without power, so a power cut stops only the active units of the others
  static const std::vector<Hardware> items = []
  {
    std::vector<Hardware> components = {
      {"arm",
       false,
       true,
       {"a1", "a2", "a3"},
       {"position", "velocity"},
       {"position", "velocity"},
       {}},
      {"base", true, true, {"wl", "wr"}, {"velocity"}, {"position", "velocity"}, {}},
      {"flipper", false, false, {"f1"}, {"position", "velocity"}, {"position"}, {}},
      {"imu", true, true, {}, {}, {}, {"imu/yaw"}},
    };
    const char * all_async = std::getenv("CM_FUZZ_ALL_ASYNC");
    if (all_async && std::string(all_async) == "1")
    {
      for (auto & component : components)
      {
        component.is_async = true;
      }
    }
    return components;
  }();
  return items;
}

const std::vector<Controller> & controllers()
{
  // Four levels from teleop down to arm, a diamond through arm_ik and wrist onto arm_vel, two
  // writers on arm_vel, competing users of diff_drive, mode conflicts on arm and flipper, a chain
  // across sync and async hardware and a reader of exported state.
  static const std::vector<Controller> items = {
    // f1/effort does not exist, best effort must skip it
    {"jsb",
     false,
     {},
     {"a1/position", "a2/position", "a3/position", "wl/position", "wr/position", "f1/position",
      "f1/effort"},
     {},
     {},
     true},
    {"arm_vel",
     true,
     {"a1/velocity", "a2/velocity", "a3/velocity"},
     {"a1/velocity", "a2/velocity", "a3/velocity"},
     {"a1", "a2", "a3"},
     {}},
    {"arm_pos", false, {"a1/position", "a2/position", "a3/position"}, {}, {}, {}},
    {"arm_ik", true, {"arm_vel/a1", "arm_vel/a2"}, {}, {"x", "z"}, {}},
    {"wrist", true, {"arm_vel/a3"}, {}, {"angle"}, {}},
    {"base_vel",
     true,
     {"wl/velocity", "wr/velocity"},
     {"wl/velocity", "wr/velocity"},
     {"wl", "wr"},
     {}},
    {"diff_drive", true, {"base_vel/wl", "base_vel/wr"}, {"imu/yaw"}, {"vx", "wz"}, {"odom"}},
    {"flipper_pos", true, {"f1/position"}, {}, {"f1"}, {}},
    {"flipper_vel", false, {"f1/velocity"}, {}, {}, {}},
    {"whole_body",
     true,
     {"arm_ik/x", "arm_ik/z", "wrist/angle", "diff_drive/vx", "diff_drive/wz", "flipper_pos/f1"},
     {},
     {"a", "b"},
     {}},
    {"nav", false, {"diff_drive/vx", "diff_drive/wz"}, {}, {}, {}},
    {"teleop", false, {"whole_body/a", "whole_body/b"}, {}, {}, {}},
    {"odom_pub", false, {}, {"diff_drive/odom", "imu/yaw"}, {}, {}},
    {"monitor", false, {}, {}, {}, {}},
  };
  return items;
}

const StartupProfile & startup_profile()
{
  static const StartupProfile profile = {
    {"arm", "base", "flipper", "imu"},
    {"jsb", "arm_vel", "arm_pos", "arm_ik", "wrist", "base_vel", "diff_drive", "flipper_pos",
     "flipper_vel", "whole_body", "nav", "teleop", "odom_pub", "monitor"},
    {"jsb", "arm_vel", "arm_ik", "wrist", "base_vel", "diff_drive", "flipper_pos", "odom_pub"},
  };
  return profile;
}

std::optional<uint16_t> hardware_index(std::string_view name)
{
  return index_by_name(hardware(), name);
}

std::optional<uint16_t> controller_index(std::string_view name)
{
  return index_by_name(controllers(), name);
}

const Controller & controller(std::string_view name)
{
  const auto index = controller_index(name);
  if (!index)
  {
    throw std::invalid_argument("Unknown rig controller " + std::string(name));
  }
  return controllers()[*index];
}

const char * plugin_type(const Controller & controller)
{
  return controller.chainable ? TRACE_CHAINABLE_CONTROLLER : TRACE_CONTROLLER;
}

namespace
{
std::string urdf_of(const std::vector<Hardware> & components)
{
  std::string xml = "<?xml version=\"1.0\"?><robot name=\"cm_test_rig\"><link name=\"base_link\"/>";
  for (const auto & hw : hardware())
  {
    const std::string limits = continuous(hw) ? std::string()
                                              : "lower=\"" + number(-POSITION_LIMIT) +
                                                  "\" upper=\"" + number(POSITION_LIMIT) + "\" ";
    for (const auto & joint : hw.joints)
    {
      xml += "<link name=\"link_" + joint + "\"/><joint name=\"" + joint + "\" type=\"" +
             (continuous(hw) ? "continuous" : "revolute") +
             "\"><parent link=\"base_link\"/><child link=\"link_" + joint +
             "\"/><axis xyz=\"0 0 1\"/><limit " + limits + "velocity=\"" + number(VELOCITY_LIMIT) +
             "\" effort=\"1\"/></joint>";
    }
  }
  for (const auto & hw : components)
  {
    xml += hardware_xml(hw);
  }
  return xml + "</robot>";
}
}  // namespace

std::string urdf() { return urdf_of(hardware()); }

std::string urdf_with_duplicate_interfaces()
{
  auto components = hardware();
  components.at(*hardware_index("flipper")).joints.push_back("a1");
  return urdf_of(components);
}

std::optional<uint16_t> interface_id(std::string_view full_name)
{
  const auto & table = interface_table();
  const auto it = std::find(table.begin(), table.end(), full_name);
  if (it == table.end())
  {
    return std::nullopt;
  }
  return static_cast<uint16_t>(it - table.begin());
}

const std::string & interface_name(uint16_t id) { return interface_table().at(id); }

size_t interface_count() { return interface_table().size(); }

bool hardware_state_exists(std::string_view full_name)
{
  for (const auto & hw : hardware())
  {
    for (const auto & joint : hw.joints)
    {
      for (const auto & state : hw.joint_states)
      {
        if (full_name == joint + "/" + state)
        {
          return true;
        }
      }
    }
    for (const auto & state : hw.sensor_states)
    {
      if (full_name == state)
      {
        return true;
      }
    }
  }
  return false;
}

std::optional<std::string> hardware_of_interface(std::string_view full_name)
{
  const auto device = prefix(full_name);
  for (const auto & hw : hardware())
  {
    const bool is_joint = std::find(hw.joints.begin(), hw.joints.end(), device) != hw.joints.end();
    const bool is_sensor = std::any_of(
      hw.sensor_states.begin(), hw.sensor_states.end(),
      [&](const std::string & state) { return prefix(state) == device; });
    if (is_joint || is_sensor)
    {
      return hw.name;
    }
  }
  return std::nullopt;
}

Names hardware_of(std::string_view controller_name)
{
  const auto & c = controller(controller_name);
  Names result = commanded_hardware_of(controller_name);
  for (const auto & name : c.state_interfaces)
  {
    if (auto hw = hardware_of_interface(name))
    {
      result.insert(*hw);
    }
  }
  return result;
}

Names commanded_hardware_of(std::string_view controller_name)
{
  Names result;
  for (const auto & name : controller(controller_name).command_interfaces)
  {
    if (auto hw = hardware_of_interface(name))
    {
      result.insert(*hw);
    }
  }
  return result;
}

Names providers(std::string_view controller_name)
{
  const auto & c = controller(controller_name);
  Names result;
  for (const auto * interfaces : {&c.command_interfaces, &c.state_interfaces})
  {
    for (const auto & name : *interfaces)
    {
      if (!hardware_of_interface(name))
      {
        result.insert(std::string(prefix(name)));
      }
    }
  }
  return result;
}

bool limit_probed(const Hardware & hw, std::string_view mode)
{
  return !(hw.is_async && mode == "velocity" && !continuous(hw));
}

Names written_providers(std::string_view controller_name)
{
  Names result;
  for (const auto & name : controller(controller_name).command_interfaces)
  {
    if (!hardware_of_interface(name))
    {
      result.insert(std::string(prefix(name)));
    }
  }
  return result;
}

Names users(std::string_view controller_name)
{
  Names result;
  for (const auto & c : controllers())
  {
    if (
      claims_any(c.command_interfaces, controller_name) ||
      claims_any(c.state_interfaces, controller_name))
    {
      result.insert(c.name);
    }
  }
  return result;
}

Names reference_writers(std::string_view controller_name)
{
  Names result;
  for (const auto & c : controllers())
  {
    if (claims_any(c.command_interfaces, controller_name))
    {
      result.insert(c.name);
    }
  }
  return result;
}

Names claim_conflicts(std::string_view controller_name)
{
  const auto & own = controller(controller_name).command_interfaces;
  Names result;
  for (const auto & c : controllers())
  {
    const bool shares =
      c.name != controller_name &&
      std::any_of(
        c.command_interfaces.begin(), c.command_interfaces.end(), [&](const std::string & name)
        { return std::find(own.begin(), own.end(), name) != own.end(); });
    if (shares)
    {
      result.insert(c.name);
    }
  }
  return result;
}

Names command_conflicts(std::string_view controller_name)
{
  Names result = claim_conflicts(controller_name);
  const Names joints = joints_commanded_by(controller_name);
  for (const auto & c : controllers())
  {
    const Names other = joints_commanded_by(c.name);
    const bool shares = std::any_of(
      other.begin(), other.end(), [&](const std::string & joint) { return joints.count(joint); });
    if (c.name != controller_name && shares)
    {
      result.insert(c.name);
    }
  }
  return result;
}

Names closure(const Names & start, const std::function<Names(std::string_view)> & relation)
{
  Names result;
  std::vector<std::string> pending(start.begin(), start.end());
  while (!pending.empty())
  {
    const std::string name = pending.back();
    pending.pop_back();
    for (const auto & next : relation(name))
    {
      if (result.insert(next).second)
      {
        pending.push_back(next);
      }
    }
  }
  return result;
}

Names connected(const Names & start)
{
  Names result = closure(
    start,
    [](std::string_view name)
    {
      Names neighbours = providers(name);
      neighbours.merge(users(name));
      return neighbours;
    });
  result.insert(start.begin(), start.end());
  return result;
}

}  // namespace cm_test_suite::rig
