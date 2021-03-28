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

#include <miniv_control/miniv_driver.hpp>

namespace miniv_control
{
MiniVDriver::MiniVDriver(
  const std::string & serial_port_name,
  const int & baudrate,
  const uint8_t & left_dynamixel_id,
  const uint8_t & right_dynamixel_id)
: without_dynamixel(false),
  serial_port_name(serial_port_name),
  baudrate(baudrate),
  left_dynamixel_id(left_dynamixel_id),
  right_dynamixl_id(right_dynamixl_id)
{
}

void MiniVDriver::openDynamixelPort() const
{
  if (!dxl_port_handler_->openPort()) {
    throw std::runtime_error(
            std::string(__func__) + ": unable to open dynamixel port: " +
            dxl_port_handler_->getPortName());
  }
  if (!dxl_port_handler_->setBaudRate(baudrate)) {
    throw std::runtime_error(
            std::string(__func__) + ": unable to set baudrate" +
            std::to_string(dxl_port_handler_->getBaudRate()));
  }
}
}  // namespace miniv_control
