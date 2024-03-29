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

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <limits>
#include <memory>
#include <miniv_control/constants.hpp>
#include <miniv_control/miniv_driver.hpp>
#include <miniv_control/miniv_hardware.hpp>
#include <string>
#include <vector>

namespace miniv_control
{
MiniVHardware::~MiniVHardware() {}

#if defined(GALACTIC) || defined(HUMBLE)
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MiniVHardware::on_init(
  const hardware_interface::HardwareInfo & info)
#else
return_type MiniVHardware::configure(const hardware_interface::HardwareInfo & info)
#endif
{
#if defined(GALACTIC) || defined(HUMBLE)
  if (
    SystemInterface::on_init(info) !=
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
#else
  if (configure_default(info) != hardware_interface::return_type::OK) {
#if defined(GALACTIC) || defined(HUMBLE)
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
#else
    return return_type::ERROR;
#endif
  }
#endif
  left_thrust_cmd_ = 0;
  right_thrust_cmd_ = 0;
  std::string thruster_ip_address = info_.hardware_parameters["ip_address"];
  int thruster_port = std::stoi(info_.hardware_parameters["port"]);
  left_thruster_joint_ = info_.hardware_parameters["left_thruster_joint"];
  right_thruster_joint_ = info_.hardware_parameters["right_thruster_joint"];
  bool enable_dummy = false;
  if (
    info_.hardware_parameters["enable_dummy"] == "true" ||
    info_.hardware_parameters["enable_dummy"] == "True") {
    enable_dummy = true;
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger("MiniVHardware"), "Connecting to motor driver...");
  RCLCPP_INFO_STREAM(rclcpp::get_logger("MiniVHardware"), "IP Address : " << thruster_ip_address);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("MiniVHardware"), "Port : " << thruster_port);
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("MiniVHardware"), "Left Thruster Joint : " << left_thruster_joint_);
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("MiniVHardware"), "Right Thruster Joint : " << right_thruster_joint_);
  try {
    driver_ = std::make_shared<MiniVDriver>(thruster_ip_address, thruster_port, enable_dummy);
  } catch (const std::runtime_error & e) {
#if defined(GALACTIC) || defined(HUMBLE)
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
#else
    return return_type::ERROR;
#endif
  }
#if defined(GALACTIC) || defined(HUMBLE)
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
#else
  return return_type::OK;
#endif
}

std::vector<hardware_interface::StateInterface> MiniVHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces = {};
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    left_thruster_joint_, hardware_interface::HW_IF_VELOCITY, &left_thrust_cmd_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    right_thruster_joint_, hardware_interface::HW_IF_VELOCITY, &right_thrust_cmd_));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MiniVHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces = {};
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    left_thruster_joint_, hardware_interface::HW_IF_VELOCITY, &left_thrust_cmd_));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    right_thruster_joint_, hardware_interface::HW_IF_VELOCITY, &right_thrust_cmd_));
  return command_interfaces;
}

// #ifndef GALACTIC
// return_type MiniVHardware::start()
// {
//   status_ = hardware_interface::status::STARTED;
//   return return_type::OK;
// }

// return_type MiniVHardware::stop()
// {
//   status_ = hardware_interface::status::STOPPED;
//   return return_type::OK;
// }
// #endif

return_type MiniVHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // RCLCPP_INFO_STREAM(rclcpp::get_logger("MiniVHardware"), __FILE__ << "," << __LINE__);
  return return_type::OK;
}

return_type MiniVHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  driver_->setThrust(Motor::THRUSTER_LEFT, left_thrust_cmd_);
  driver_->setThrust(Motor::TURUSTER_RIGHT, right_thrust_cmd_);
  if (driver_->sendCommand()) {
    return return_type::OK;
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("MiniVHardware"), "failed to send command.");
    return return_type::ERROR;
  }
}
}  // namespace miniv_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(miniv_control::MiniVHardware, hardware_interface::SystemInterface)
