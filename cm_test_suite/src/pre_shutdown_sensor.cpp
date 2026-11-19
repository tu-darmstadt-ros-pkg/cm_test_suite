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

#include "hardware_interface/sensor_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pre_shutdown_hook.hpp"

namespace cm_test_suite
{
/// A sensor whose init registers the hook, the hardware variant of PreShutdownController.
class PreShutdownSensor : public hardware_interface::SensorInterface
{
public:
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    if (SensorInterface::on_init(params) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }
    register_pre_shutdown_hook();
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    return hardware_interface::return_type::OK;
  }
};
}  // namespace cm_test_suite

PLUGINLIB_EXPORT_CLASS(cm_test_suite::PreShutdownSensor, hardware_interface::SensorInterface)
