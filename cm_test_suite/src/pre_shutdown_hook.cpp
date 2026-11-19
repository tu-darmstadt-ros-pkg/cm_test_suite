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

#include "pre_shutdown_hook.hpp"

#include "rclcpp/contexts/default_context.hpp"

namespace cm_test_suite
{
void register_pre_shutdown_hook()
{
  rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback([] {});
}
}  // namespace cm_test_suite
