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
  const int & thruster_port,
  const std::string & dynamixel_port_name,
  const int & baudrate,
  const uint8_t & left_dynamixel_id,
  const uint8_t & right_dynamixel_id)
: thruster_ip_address(thruster_ip_address),
  thruster_port(thruster_port),
  without_dynamixel(false),
  dynamixel_port_name(dynamixel_port_name),
  baudrate(baudrate),
  left_dynamixel_id(left_dynamixel_id),
  right_dynamixel_id(right_dynamixel_id)
{
  dynamixel_port_handler_ = std::shared_ptr<dynamixel::PortHandler>(
    dynamixel::PortHandler::getPortHandler(dynamixel_port_name.c_str()));
  dynamixel_packet_handler_ = std::shared_ptr<dynamixel::PacketHandler>(
    dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION));
  openDynamixelPort();
  torqueEnable(Motor::ALL, true);
  boost::asio::io_service io_service;
  tcp_client_ = std::make_unique<tcp_sender::TcpClient>(
    io_service, rclcpp::get_logger("MiniVHardware"));
  tcp_client_->connect(thruster_ip_address, thruster_port);
}

MiniVDriver::MiniVDriver(
  const std::string & thruster_ip_address,
  const int & thruster_port)
: thruster_ip_address(thruster_ip_address),
  thruster_port(thruster_port),
  without_dynamixel(true),
  dynamixel_port_name(""),
  baudrate(0),
  left_dynamixel_id(0),
  right_dynamixel_id(0)
{
  boost::asio::io_service io_service;
  tcp_client_ = std::make_unique<tcp_sender::TcpClient>(
    io_service, rclcpp::get_logger("MiniVHardware"));
  tcp_client_->connect(thruster_ip_address, thruster_port);
}

MiniVDriver::~MiniVDriver()
{
  if (!without_dynamixel) {
    closeDynamixelPort();
  }
}

bool MiniVDriver::checkDynamixelError(
  const int dynamixel_comm_result, const uint8_t dynamixel_packet_error)
{
  if (dynamixel_comm_result != COMM_SUCCESS || dynamixel_packet_error != 0) {
    return false;
  }
  return true;
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

boost::optional<double> MiniVDriver::getCurrentAngle(const Motor & motor)
{
  switch (motor) {
    case Motor::AZIMUTH_LEFT:
      return getCurrentAngle(left_dynamixel_id);
      break;
    case Motor::AZIMUTH_RIGHT:
      return getCurrentAngle(right_dynamixel_id);
    default:
      break;
  }
  return boost::none;
}

std::unordered_map<Motor, double> MiniVDriver::getCurrentAngle()
{
  std::unordered_map<Motor, double> ret;
  if(const auto angle = getCurrentAngle(Motor::AZIMUTH_LEFT)) {
    ret[Motor::AZIMUTH_LEFT] = angle.get();
  }
  if(const auto angle = getCurrentAngle(Motor::AZIMUTH_RIGHT)) {
    ret[Motor::AZIMUTH_RIGHT] = angle.get();
  }
  return ret;
}

bool MiniVDriver::setGoalAngle(uint8_t id, const double & goal_angle)
{
  uint8_t dynamixel_error = 0;
  uint16_t goal_position = radianToDynamixelPosition(goal_angle);
  return dynamixel_packet_handler_->write2ByteTxRx(
    dynamixel_port_handler_.get(), id, ADDR_GOAL_POSITION, goal_position, &dynamixel_error);
}

bool MiniVDriver::setGoalAngle(const Motor & motor, const double & goal_angle)
{
  switch (motor) {
    case Motor::AZIMUTH_LEFT:
      return setGoalAngle(left_dynamixel_id, goal_angle);
      break;
    case Motor::AZIMUTH_RIGHT:
      return setGoalAngle(right_dynamixel_id, goal_angle);
      break;
    case Motor::AZIMUTH:
      if (setGoalAngle(left_dynamixel_id, goal_angle) &&
        setGoalAngle(right_dynamixel_id, goal_angle))
      {
        return true;
      }
      return false;
      break;
    default:
      break;
  }
  return false;
}

bool MiniVDriver::torqueEnable(const Motor & motor, bool enable)
{
  switch (motor) {
    case Motor::AZIMUTH_LEFT:
      return torqueEnable(enable, left_dynamixel_id);
      break;
    case Motor::AZIMUTH_RIGHT:
      return torqueEnable(enable, right_dynamixel_id);
      break;
    case Motor::AZIMUTH:
      if (torqueEnable(enable, left_dynamixel_id) && torqueEnable(enable, right_dynamixel_id)) {
        return true;
      }
      return false;
      break;
    default:
      break;
  }
  return false;
}

bool MiniVDriver::torqueEnable(bool enable, uint8_t id)
{
  uint8_t dynamixel_error = 0;
  int dynamixel_result = dynamixel_packet_handler_->write1ByteTxRx(
    dynamixel_port_handler_.get(),
    id, ADDR_TORQUE_ENABLE, enable, &dynamixel_error);
  if (!checkDynamixelError(dynamixel_result, dynamixel_error)) {
    return false;
  }
  return true;
}

boost::optional<double> MiniVDriver::getCurrentAngle(uint8_t id)
{
  uint8_t dynamixel_error = 0;
  uint16_t dynamixel_present_position = 0;
  int dynamixel_result = dynamixel_packet_handler_->read2ByteTxRx(
    dynamixel_port_handler_.get(),
    id, ADDR_PRESENT_POSITION, &dynamixel_present_position, &dynamixel_error);
  if (!checkDynamixelError(dynamixel_result, dynamixel_error)) {
    return boost::none;
  }
  return dynamixelPositionToRadian(dynamixel_present_position);
}

void MiniVDriver::closeDynamixelPort() const
{
  dynamixel_port_handler_->closePort();
}

double MiniVDriver::dynamixelPositionToRadian(const uint16_t position) const
{
  return (position - DXL_HOME_POSITION) * TO_RADIANS;
}

uint16_t MiniVDriver::radianToDynamixelPosition(const double position) const
{
  return position * TO_DXL_POS + DXL_HOME_POSITION;
}

bool MiniVDriver::openDynamixelPort() const
{
  if (!dynamixel_port_handler_->openPort()) {
    return false;
  }
  if (!dynamixel_port_handler_->setBaudRate(baudrate)) {
    return false;
  }
  return true;
}
}  // namespace miniv_control
