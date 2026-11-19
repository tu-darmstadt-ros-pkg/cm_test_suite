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

#ifndef CM_TEST_SUITE__SUITE_HPP_
#define CM_TEST_SUITE__SUITE_HPP_

#include <string>

#include "cm_test_suite/cm_under_test.hpp"

/// The common test suite, registered with gtest for one controller manager implementation.
///
/// Registers the acceptance scripts installed with this package, a stepped and a threaded fuzzer
/// and, if set, the script in CM_FUZZ_SCRIPT. Environment variables:
///   CM_FUZZ_SEED           first seed (default 0)
///   CM_FUZZ_SEEDS          stepped fuzz seeds (default 20)
///   CM_FUZZ_THREADED_SEEDS threaded fuzz seeds (default 5)
///   CM_FUZZ_STEPS          steps per seed (default 100)
///   CM_FUZZ_SHRINK         shrink a failing fuzz script before printing it (default 1)
///   CM_FUZZ_SHRINK_SECONDS time budget of one shrink (default 60)
///   CM_FUZZ_SKIP           comma separated check categories to ignore, see runner.hpp
///   CM_FUZZ_TIER           "gates" reports only hard gate failures, as for judging
///   CM_FUZZ_REQUIRE_ALL    1 fails scripts needing a capability the controller manager lacks
///   CM_FUZZ_SCRIPT         path of a script to run, as printed by a failing fuzz test
///   CM_FUZZ_ASYNC_GRACE_CYCLES cycles async hardware may apply a stopped writer's command
///                          (default 1), 0 for a controller manager that holds it first
///   CM_FUZZ_ALL_ASYNC      1 makes every rig component async
namespace cm_test_suite
{
void register_suite(const std::string & name, const Capabilities & capabilities, CmFactory factory);

}  // namespace cm_test_suite

#endif  // CM_TEST_SUITE__SUITE_HPP_
