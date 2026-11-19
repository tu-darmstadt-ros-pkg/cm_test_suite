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

#ifndef NAMES_HPP_
#define NAMES_HPP_

#include <algorithm>
#include <iterator>
#include <string>
#include <vector>

#include "cm_test_suite/oracle.hpp"

/// Set helpers and snapshot accessors shared by the oracle files.
namespace cm_test_suite::oracle
{
inline Names merged(Names a, const Names & b)
{
  a.insert(b.begin(), b.end());
  return a;
}

inline Names without(Names a, const Names & b)
{
  for (const auto & name : b)
  {
    a.erase(name);
  }
  return a;
}

inline Names intersection(const Names & a, const Names & b)
{
  Names result;
  std::set_intersection(
    a.begin(), a.end(), b.begin(), b.end(), std::inserter(result, result.begin()));
  return result;
}

inline bool is_subset(const Names & a, const Names & b)
{
  return std::includes(b.begin(), b.end(), a.begin(), a.end());
}

inline Names from(const std::vector<std::string> & names) { return {names.begin(), names.end()}; }

inline std::string text(const Names & names)
{
  std::string result = "[";
  for (const auto & name : names)
  {
    result += (result.size() > 1 ? " " : "") + name;
  }
  return result + "]";
}

inline const ControllerTruth & controller_truth(const Snapshot & snapshot, const std::string & name)
{
  return snapshot.controllers.at(*rig::controller_index(name));
}

inline Lifecycle controller_state(const Snapshot & snapshot, const std::string & name)
{
  return controller_truth(snapshot, name).state;
}

inline const HardwareTruth & hardware_truth(const Snapshot & snapshot, const std::string & name)
{
  return snapshot.hardware.at(*rig::hardware_index(name));
}

inline Lifecycle hardware_state(const Snapshot & snapshot, const std::string & name)
{
  return hardware_truth(snapshot, name).state;
}

/// The controller may run on the hardware in this state. Hardware it commands must be active,
/// hardware it only reads may also be inactive or, for a best effort reader, down.
inline bool hardware_usable(
  const std::string & controller, const std::string & hardware, const Snapshot & snapshot)
{
  const auto state = hardware_state(snapshot, hardware);
  return state == Lifecycle::Active ||
         (!rig::commanded_hardware_of(controller).count(hardware) &&
          (state == Lifecycle::Inactive || rig::controller(controller).best_effort_states));
}

}  // namespace cm_test_suite::oracle

#endif  // NAMES_HPP_
