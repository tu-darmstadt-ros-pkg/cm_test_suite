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

// Runs with CM_FUZZ_ALL_ASYNC=1, which the rig reads once per process

#include <gtest/gtest.h>

#include "cm_test_suite/rig.hpp"
#include "cm_test_suite/trace.hpp"
#include "controller_tracer.hpp"

namespace cm_test_suite
{
TEST(Limits, controllers_do_not_probe_the_velocity_of_an_async_arm_with_position_limits)
{
  ASSERT_TRUE(rig::hardware()[*rig::hardware_index("arm")].is_async);
  trace().reset();
  const auto probes = [](const char * name)
  {
    ControllerTracer controller(name);
    size_t count = 0;
    for (uint64_t i = 0; i < 4 * rig::LIMIT_PROBE_PERIOD; ++i)
    {
      count += controller.mint_command(0, 0) == rig::LIMIT_PROBE;
    }
    return count;
  };
  EXPECT_EQ(0u, probes("arm_vel"));
  EXPECT_GE(probes("arm_pos"), 3u);
}
}  // namespace cm_test_suite
