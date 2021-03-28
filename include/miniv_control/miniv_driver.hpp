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

#include <string>
#include <vector>
#include <memory>

namespace miniv_control
{
class MiniVDriver
{
public:
  explicit MiniVDriver(
    const std::string & dynamixel_port_name,
    const int & baudrate,
    const uint8_t & left_dynamixel_id,
    const uint8_t & right_dynamixel_id);
  const bool without_dynamixel;
  const std::string dynamixel_port_name;
  const int baudrate;
  const uint8_t left_dynamixel_id;
  const uint8_t right_dynamixl_id;

private:
  void openDynamixelPort() const;
  std::shared_ptr<dynamixel::PortHandler> dxl_port_handler_;
  std::shared_ptr<dynamixel::PacketHandler> dxl_packet_handler_;
};
}  // namespace miniv_control

#endif  // MINIV_CONTROL__MINIV_DRIVER_HPP_
