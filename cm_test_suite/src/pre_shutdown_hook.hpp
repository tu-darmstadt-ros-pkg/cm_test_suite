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

#ifndef PRE_SHUTDOWN_HOOK_HPP_
#define PRE_SHUTDOWN_HOOK_HPP_

namespace cm_test_suite
{
/// Registers a pre-shutdown callback on the default context and never removes it. The callback's
/// code lives in this library, which only the plugin links, like libmoveit_utils.
void register_pre_shutdown_hook();
}  // namespace cm_test_suite

#endif  // PRE_SHUTDOWN_HOOK_HPP_
