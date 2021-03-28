// Copyright (c) 2019 OUXT Polaris
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

#ifndef MINIV_CONTROL__MINIV_HARDWARE_HPP_
#define MINIV_CONTROL__MINIV_HARDWARE_HPP_

#include <miniv_control/visibility_control.hpp>

#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_status_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <vector>

using hardware_interface::return_type;

namespace miniv_control
{
class MiniVHardware : public
  hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MiniVHardware)

  MINIV_CONTROL_PUBLIC
  ~MiniVHardware();

  MINIV_CONTROL_PUBLIC
  return_type configure(const hardware_interface::HardwareInfo & info) override;

  MINIV_CONTROL_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  MINIV_CONTROL_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  MINIV_CONTROL_PUBLIC
  return_type start() override;

  MINIV_CONTROL_PUBLIC
  return_type stop() override;

  MINIV_CONTROL_PUBLIC
  return_type read() override;

  MINIV_CONTROL_PUBLIC
  return_type write() override;

private:
  bool communication_timeout();

  // std::shared_ptr<CranePlusDriver> driver_;
  double timeout_seconds_;

  std::vector<double> hw_position_commands_;
  std::vector<double> hw_position_states_;
  std::vector<double> hw_velocity_states_;
  std::vector<double> hw_load_states_;
  std::vector<double> hw_voltage_states_;
  std::vector<double> hw_temperature_states_;

  rclcpp::Time prev_comm_timestamp_;
};
}  // namespace miniv_control

#endif  // MINIV_CONTROL__MINIV_HARDWARE_HPP_
