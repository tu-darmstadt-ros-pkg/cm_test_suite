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

#ifndef CM_TEST_SUITE__TYPES_HPP_
#define CM_TEST_SUITE__TYPES_HPP_

#include <cstdint>

namespace cm_test_suite
{
enum class Lifecycle : uint8_t
{
  Unloaded,
  Unconfigured,
  Inactive,
  Active,
  Finalized,
};

const char * to_string(Lifecycle state);

enum class TargetKind : uint8_t
{
  Controller,
  Hardware,
};

/// A rig controller or hardware component, by its index in the rig.
struct Target
{
  TargetKind kind;
  uint16_t index;
};

}  // namespace cm_test_suite

#endif  // CM_TEST_SUITE__TYPES_HPP_
