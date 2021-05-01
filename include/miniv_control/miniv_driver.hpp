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

#ifndef MINIV_CONTROL__MINIV_DRIVER_HPP_
#define MINIV_CONTROL__MINIV_DRIVER_HPP_

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <tcp_sender/tcp_client.hpp>

#include <boost/optional.hpp>

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>

namespace miniv_control
{
enum class Motor
{
  AZIMUTH_LEFT,
  AZIMUTH_RIGHT,
  THRUSTER_LEFT,
  TURUSTER_RIGHT,
  AZIMUTH,
  THRUSTER,
  ALL
};

class MiniVDriver
{
public:
  explicit MiniVDriver(
    const std::string & thruster_ip_address,
    const int & thruster_port,
    const std::string & dynamixel_port_name,
    const int & baudrate,
    const uint8_t & left_dynamixel_id,
    const uint8_t & right_dynamixel_id);
  MiniVDriver(
    const std::string & thruster_ip_address,
    const int & thruster_port);
  ~MiniVDriver();

  const std::string thruster_ip_address;
  const int thruster_port;
  const bool without_dynamixel;
  const std::string dynamixel_port_name;
  const int baudrate;
  const uint8_t left_dynamixel_id;
  const uint8_t right_dynamixel_id;
  bool torqueEnable(const Motor & motor, bool enable);
  bool setGoalAngle(const Motor & motor, const double & goal_angle);
  boost::optional<double> getCurrentAngle(const Motor & motor);
  std::unordered_map<Motor, double> getCurrentAngle();
  bool setThrust(const Motor & motor, double thrust);

private:
  double left_thrust_;
  double right_thrust_;
  std::unique_ptr<tcp_sender::TcpClient> tcp_client_;
  bool setGoalAngle(uint8_t id, const double & goal_angle);
  bool torqueEnable(bool enable, uint8_t id);
  boost::optional<double> getCurrentAngle(uint8_t id);
  bool checkDynamixelError(
    const int dynamixel_comm_result, const uint8_t dynamixel_packet_error);
  bool openDynamixelPort() const;
  void closeDynamixelPort() const;
  std::shared_ptr<dynamixel::PortHandler> dynamixel_port_handler_;
  std::shared_ptr<dynamixel::PacketHandler> dynamixel_packet_handler_;
  double dynamixelPositionToRadian(const uint16_t position) const;
  uint16_t radianToDynamixelPosition(const double position) const;
};
}  // namespace miniv_control

#endif  // MINIV_CONTROL__MINIV_DRIVER_HPP_
