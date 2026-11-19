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

#ifndef EVENT_SUMMARY_HPP_
#define EVENT_SUMMARY_HPP_

#include <string>
#include <vector>

#include "cm_test_suite/trace.hpp"

namespace cm_test_suite
{
struct EventSummary
{
  /// event:<kind slug>
  std::string category;
  std::string text;
};

/// One line per distinct event, counting its repetitions, in order of first occurrence.
std::vector<EventSummary> summarize(const std::vector<Event> & events);

}  // namespace cm_test_suite

#endif  // EVENT_SUMMARY_HPP_
