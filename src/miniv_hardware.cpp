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

#include <miniv_control/miniv_hardware.hpp>
#include <miniv_control/miniv_driver.hpp>
#include <miniv_control/constants.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <string>
#include <memory>
#include <limits>
#include <vector>

namespace miniv_control
{
MiniVHardware::~MiniVHardware()
{
}

return_type MiniVHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }
  std::string thruster_ip_address = info_.hardware_parameters["thruster_ip_address"];
  int thruster_port = std::stoi(info_.hardware_parameters["port"]);
  try {
    driver_ = std::make_shared<MiniVDriver>(
      thruster_ip_address, thruster_port);
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(rclcpp::get_logger("MiniVHardware"), e.what());
    return return_type::ERROR;
  }
  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface>
MiniVHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces = {};
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MiniVHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces = {};
  return command_interfaces;
}

return_type MiniVHardware::start()
{
  status_ = hardware_interface::status::STARTED;
  return return_type::OK;
}

return_type MiniVHardware::stop()
{
  status_ = hardware_interface::status::STOPPED;
  return return_type::OK;
}

return_type MiniVHardware::read()
{
  return return_type::OK;
}

return_type MiniVHardware::write()
{
  return return_type::OK;
}
}  // namespace miniv_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(miniv_control::MiniVHardware, hardware_interface::SystemInterface)
