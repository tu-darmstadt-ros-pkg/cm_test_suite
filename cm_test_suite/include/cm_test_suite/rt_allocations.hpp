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

#ifndef CM_TEST_SUITE__RT_ALLOCATIONS_HPP_
#define CM_TEST_SUITE__RT_ALLOCATIONS_HPP_

#include <cstdint>

/// Counts heap allocations made through operator new on a real-time thread.
/// Linking this replaces the global operator new, except in sanitizer builds that need their own.
namespace cm_test_suite
{
bool rt_allocation_counting_available();

/// Allocations made inside any RtScope so far.
uint64_t rt_allocations();

/// Marks the calling thread as real-time while alive.
class RtScope
{
public:
  RtScope();
  ~RtScope();

  RtScope(const RtScope &) = delete;
  RtScope & operator=(const RtScope &) = delete;
};

}  // namespace cm_test_suite

#endif  // CM_TEST_SUITE__RT_ALLOCATIONS_HPP_
