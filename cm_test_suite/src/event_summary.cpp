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

#include "event_summary.hpp"

#include <algorithm>
#include <tuple>
#include <utility>

#include "cm_test_suite/rig.hpp"

namespace cm_test_suite
{
namespace
{
std::string subject_name(const Target & subject)
{
  if (subject.kind == TargetKind::Controller)
  {
    return subject.index < rig::controllers().size()
             ? "controller " + rig::controllers()[subject.index].name
             : "unknown controller";
  }
  return subject.index < rig::hardware().size() ? "hardware " + rig::hardware()[subject.index].name
                                                : "unknown hardware";
}

/// One line for all repetitions of the same event.
std::string describe(const Event & event, size_t count, uint64_t last_cycle)
{
  std::string text = subject_name(event.subject) + ": " + to_string(event.kind);
  if (event.interface != NO_INDEX)
  {
    text += " in " + rig::interface_name(event.interface);
  }
  if (event.writer != NO_INDEX)
  {
    text += " concerning " + rig::controllers().at(event.writer).name;
  }
  text += " (cycle " + std::to_string(event.cycle);
  if (count > 1)
  {
    text += " to " + std::to_string(last_cycle) + ", " + std::to_string(count) + " times";
  }
  return text + ")";
}
}  // namespace

std::vector<EventSummary> summarize(const std::vector<Event> & events)
{
  using Key = std::tuple<EventKind, TargetKind, uint16_t, uint16_t, uint16_t>;
  struct Repeated
  {
    Event first;
    size_t count;
    uint64_t last_cycle;
  };
  std::vector<std::pair<Key, Repeated>> grouped;
  for (const auto & event : events)
  {
    const Key key{
      event.kind, event.subject.kind, event.subject.index, event.interface, event.writer};
    const auto it = std::find_if(
      grouped.begin(), grouped.end(), [&](const auto & entry) { return entry.first == key; });
    if (it == grouped.end())
    {
      grouped.push_back({key, {event, 1, event.cycle}});
    }
    else
    {
      ++it->second.count;
      it->second.last_cycle = event.cycle;
    }
  }
  std::vector<EventSummary> summaries;
  for (const auto & [key, repeated] : grouped)
  {
    summaries.push_back(
      {std::string("event:") + slug(repeated.first.kind),
       describe(repeated.first, repeated.count, repeated.last_cycle)});
  }
  return summaries;
}

}  // namespace cm_test_suite
