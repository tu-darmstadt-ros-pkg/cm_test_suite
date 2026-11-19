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

#ifndef CM_TEST_SUITE__SHRINK_HPP_
#define CM_TEST_SUITE__SHRINK_HPP_

#include <chrono>
#include <cstddef>
#include <functional>
#include <string>

#include "cm_test_suite/script.hpp"

namespace cm_test_suite
{
/// Removes steps, then faults, stalls, holds and single list entries of the remaining steps from a
/// failing script while it keeps failing. A candidate counts as failing if one of two runs fails.
/// Calls fails at most max_runs times and starts no call after budget, then returns the smallest
/// failing script found.
Script shrink(
  const Script & script, const std::function<bool(const Script &)> & fails, size_t max_runs,
  std::chrono::steady_clock::duration budget);

/// Tier, category, phase and message of a failure with the script line, the numbers and the
/// entries of name lists dropped, so a shrunk script keeps the same failure although its steps,
/// cycles, counts and bystanders change.
std::string failure_signature(const std::string & failure);

/// Reports failure as a gtest failure at once, so a long shrink is never silent, then shrinks the
/// script and reports the result as a second failure.
void report_and_shrink(
  const std::string & failure, const Script & script,
  const std::function<bool(const Script &)> & fails, size_t max_runs,
  std::chrono::steady_clock::duration budget);

}  // namespace cm_test_suite

#endif  // CM_TEST_SUITE__SHRINK_HPP_
