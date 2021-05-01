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

#include <miniv_control/constants.hpp>
#include <miniv_control/miniv_driver.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>

#include <string>
#include <memory>

namespace miniv_control
{
MiniVDriver::MiniVDriver(
  const std::string & thruster_ip_address,
  const int & thruster_port)
: thruster_ip_address(thruster_ip_address),
  thruster_port(thruster_port)
{
  boost::asio::io_service io_service;
  tcp_client_ = std::make_unique<tcp_sender::TcpClient>(
    io_service, rclcpp::get_logger("MiniVHardware"));
  tcp_client_->connect(thruster_ip_address, thruster_port);
}

bool MiniVDriver::setThrust(const Motor & motor, double thrust)
{
  nlohmann::json json;
  switch (motor) {
    case Motor::THRUSTER:
      left_thrust_ = thrust;
      right_thrust_ = thrust;
      break;
    case Motor::THRUSTER_LEFT:
      left_thrust_ = thrust;
      break;
    case Motor::TURUSTER_RIGHT:
      right_thrust_ = thrust;
      break;
    default:
      break;
  }
  json["left_thrust"] = left_thrust_;
  json["right_thrust"] = right_thrust_;
  std::string message = json.dump();
  return tcp_client_->send(message);
}
}  // namespace miniv_control
