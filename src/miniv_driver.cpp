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

#include <memory>
#include <miniv_control/constants.hpp>
#include <miniv_control/miniv_driver.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace miniv_control
{
MiniVDriver::MiniVDriver(
  const std::string & thruster_ip_address, const int & thruster_port, bool enable_dummy)
: thruster_ip_address(thruster_ip_address), thruster_port(thruster_port), enable_dummy(enable_dummy)
{
  if (!enable_dummy) {
    boost::asio::io_service io_service;
    tcp_client_ =
      std::make_unique<tcp_sender::TcpClient>(io_service, rclcpp::get_logger("MiniVHardware"));
    tcp_client_->connect(thruster_ip_address, thruster_port);
  }
}

bool MiniVDriver::sendCommand()
{
  nlohmann::json json;
  json["left"] = left_thrust_;
  json["right"] = right_thrust_;
  std::string message = json.dump();
  // RCLCPP_INFO_STREAM(rclcpp::get_logger("MiniVHardware"), "sending command : " << message);
  if (enable_dummy) {
    return true;
  }
  return tcp_client_->send(message);
}

bool MiniVDriver::sendCommandtoMbed(double period, double duty)
{
  const int value_num = 2;
  const uint8_t length = sizeof(double) * value_num;
  uint8_t message[4 + length] = {0};
  uint8_t * period_array = reinterpret_cast<uint8_t *>(&period);
  uint8_t * duty_array = reinterpret_cast<uint8_t *>(&duty);
  message[0] = header1;
  message[1] = header2;
  message[2] = length;
  for (unsigned int i = 0; i < sizeof(double); i++) {
    message[3 + i] = period_array[i];
    message[3 + sizeof(double) + i] = duty_array[i];
  }
  message[3 + length] = end;
  // RCLCPP_INFO_STREAM(rclcpp::get_logger("MiniVHardware"), "sending command : " << message);
  if (enable_dummy) {
    return true;
  }
  return tcp_client_->send(message, 4 + length);
}

void MiniVDriver::setThrust(const Motor & motor, double thrust)
{
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
}

}  // namespace miniv_control
