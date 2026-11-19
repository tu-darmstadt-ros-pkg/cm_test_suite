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

#include "cm_test_suite/runner.hpp"

namespace cm_test_suite
{
TEST(Tiers, safety_checks_are_gates_and_bounds_are_provisional)
{
  for (const auto * gate :
       {"event:inactive_writer", "event:limit_exceeded", "event:provider_stopped_under_user",
        "event:shared_command_interface", "consistency", "shutdown", "power_cut", "exception",
        "hang"})
  {
    EXPECT_EQ(Tier::Gate, tier_of(gate)) << gate;
  }
  for (const auto * provisional :
       {"event:stale_value", "event:activated_on_unchained_provider", "outcome", "liveness",
        "status", "activity", "timeout", "rt_blocking", "rt_allocation", "power_recovery", "setup"})
  {
    EXPECT_EQ(Tier::Provisional, tier_of(provisional)) << provisional;
  }
}

}  // namespace cm_test_suite
