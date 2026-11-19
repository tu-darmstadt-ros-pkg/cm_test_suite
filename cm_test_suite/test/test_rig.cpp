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

#include <set>
#include <string>

#include "cm_test_suite/rig.hpp"

namespace cm_test_suite::rig
{
TEST(Rig, every_claimed_interface_exists)
{
  for (const auto & c : controllers())
  {
    for (const auto & name : c.command_interfaces)
    {
      EXPECT_TRUE(interface_id(name).has_value()) << c.name << " claims unknown " << name;
    }
    for (const auto & name : c.state_interfaces)
    {
      const bool known = interface_id(name).has_value() || hardware_of_interface(name).has_value();
      EXPECT_TRUE(known) << c.name << " reads unknown " << name;
    }
  }
}

TEST(Rig, interface_ids_round_trip)
{
  const auto id = interface_id("arm_vel/a3");
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ("arm_vel/a3", interface_name(*id));
  EXPECT_FALSE(interface_id("a1/position_typo").has_value());
}

TEST(Rig, relations_follow_the_chain)
{
  EXPECT_EQ(Names({"whole_body"}), providers("teleop"));
  EXPECT_EQ(Names({"arm_ik", "wrist"}), reference_writers("arm_vel"));
  EXPECT_EQ(Names({"nav", "odom_pub", "whole_body"}), users("diff_drive"));
  EXPECT_EQ(Names({"imu"}), hardware_of("diff_drive"));
  EXPECT_EQ(Names({"arm"}), hardware_of("arm_vel"));
  EXPECT_TRUE(commanded_hardware_of("jsb").empty());
  EXPECT_EQ(Names({"arm", "base", "flipper"}), hardware_of("jsb"));
  EXPECT_TRUE(hardware_of("teleop").empty());
  EXPECT_EQ(Names({"nav"}), claim_conflicts("whole_body"));
}

TEST(Rig, chain_is_four_levels_deep)
{
  const Names below_teleop = closure({"teleop"}, providers);
  EXPECT_EQ(
    Names({"arm_ik", "arm_vel", "base_vel", "diff_drive", "flipper_pos", "whole_body", "wrist"}),
    below_teleop);
}

TEST(Rig, connected_spans_users_and_providers)
{
  const Names unit = connected({"wrist"});
  EXPECT_TRUE(unit.count("teleop"));
  EXPECT_TRUE(unit.count("odom_pub"));
  EXPECT_FALSE(unit.count("jsb"));
  EXPECT_EQ(Names({"jsb"}), connected({"jsb"}));
}

TEST(Rig, urdf_names_every_hardware_with_its_async_flag)
{
  const std::string xml = urdf();
  EXPECT_NE(std::string::npos, xml.find("name=\"base\" type=\"system\" is_async=\"true\""));
  EXPECT_NE(std::string::npos, xml.find("name=\"arm\" type=\"system\" is_async=\"false\""));
  EXPECT_NE(std::string::npos, xml.find("<sensor name=\"imu\"><state_interface name=\"yaw\"/>"));
}

TEST(Rig, joints_are_limited_beyond_every_token_and_start_at_zero)
{
  const std::string xml = urdf();
  EXPECT_NE(
    std::string::npos,
    xml.find("<limit lower=\"-9007199254740992\" upper=\"9007199254740992\" velocity=\"1e+30\""));
  EXPECT_NE(
    std::string::npos,
    xml.find("<state_interface name=\"position\"><param name=\"initial_value\">0.0</param>"));
  EXPECT_GT(LIMIT_PROBE, VELOCITY_LIMIT);
  EXPECT_GT(LIMIT_PROBE, POSITION_LIMIT);
}

TEST(Rig, wheels_are_continuous_and_async_velocity_with_position_limits_is_not_probed)
{
  const std::string xml = urdf();
  EXPECT_NE(
    std::string::npos,
    xml.find(
      "<joint name=\"wl\" type=\"continuous\"><parent link=\"base_link\"/><child "
      "link=\"link_wl\"/><axis xyz=\"0 0 1\"/><limit velocity=\"1e+30\""));
  EXPECT_NE(std::string::npos, xml.find("<joint name=\"a1\" type=\"revolute\">"));
  const Hardware wheels{"base", true, true, {"wl"}, {"velocity"}, {"position", "velocity"}, {}};
  const Hardware arm{"arm", true, true, {"a1"}, {"position", "velocity"}, {"position"}, {}};
  EXPECT_TRUE(limit_probed(wheels, "velocity"));
  EXPECT_TRUE(limit_probed(arm, "position"));
  EXPECT_FALSE(limit_probed(arm, "velocity"));
  Hardware sync_arm = arm;
  sync_arm.is_async = false;
  EXPECT_TRUE(limit_probed(sync_arm, "velocity"));
}

TEST(Rig, the_duplicate_description_exports_a1_twice_and_declares_it_once)
{
  const std::string xml = rig::urdf_with_duplicate_interfaces();
  const auto count = [&](const std::string & needle)
  {
    size_t n = 0;
    for (size_t at = xml.find(needle); at != std::string::npos; at = xml.find(needle, at + 1))
    {
      ++n;
    }
    return n;
  };
  EXPECT_EQ(2u, count("<joint name=\"a1\"><command_interface"));
  EXPECT_EQ(1u, count("<joint name=\"a1\" type="));
}

TEST(Rig, startup_profile_is_consistent)
{
  const auto & profile = startup_profile();
  for (const auto & name : profile.controllers_active)
  {
    EXPECT_TRUE(profile.controllers_loaded.count(name)) << name;
    for (const auto & provider : providers(name))
    {
      EXPECT_TRUE(profile.controllers_active.count(provider)) << name << " needs " << provider;
    }
  }
}

}  // namespace cm_test_suite::rig
