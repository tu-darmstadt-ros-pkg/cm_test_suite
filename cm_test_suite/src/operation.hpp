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

#ifndef OPERATION_HPP_
#define OPERATION_HPP_

#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "control_loop.hpp"

namespace cm_test_suite
{
/// Something the harness does a number of cycles into an operation, such as arming a fault.
struct TimedAction
{
  uint64_t after_cycles;
  std::function<void()> action;
};

struct OperationResult
{
  /// Empty if the operation threw or did not return.
  std::optional<bool> accepted;
  ControlLoop::Clock::duration duration{};
  /// Why accepted is empty.
  std::string error;
  bool hung = false;
};

/// Runs a blocking operation on its own thread while the control loop keeps cycling, and runs the
/// timed actions on the way. Aborts the process if the operation never returns, because its
/// thread still references the controller manager.
OperationResult run_operation(
  ControlLoop & loop, const std::function<bool()> & operation, std::vector<TimedAction> actions,
  ControlLoop::Clock::duration hang_limit);

}  // namespace cm_test_suite

#endif  // OPERATION_HPP_
