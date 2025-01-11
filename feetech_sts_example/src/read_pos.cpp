// Copyright 2023 Ar-Ray-code.
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
#include <cstdio>
#include <memory>
#include <string>
#include <rclcpp_components/register_node_macro.hpp>

#include <feetech_sts_interface/feetech_sts_interface.hpp>
#include <h6x_serial_interface/port_handler.hpp>
#include <unistd.h>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "feetech_sts_example/read_pos.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position_six_motor.hpp"

namespace feetech_sts_interface
{

PubFeetechNode::PubFeetechNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pub_feetech_node", options)
{
  RCLCPP_INFO_STREAM(get_logger(), "start initializing publisher");
  publisher_six_motor_present_position = this->create_publisher<SetPositionSixMotor>("six_motor_presentposition", 10);
  
  int TARGET_ID = 1;
  std::string port_name = "/dev/ttyUSB0";
  int baudrate = 1000000;

  auto port_handler = std::make_shared<h6x_serial_interface::PortHandler>(port_name);
  auto packet_handler = std::make_shared<feetech_sts_interface::PacketHandler>(port_handler);

  port_handler->configure(baudrate);
  if (!port_handler->open()) {
    throw std::runtime_error("Failed to open port.");
  }

  packet_handler->setTorque(TARGET_ID, 0);

  using namespace std::chrono_literals;  // NOLINT
  timer_ = create_wall_timer(100ms,[this,packet_handler](){
    dynamixel_sdk_custom_interfaces::msg::SetPositionSixMotor msg;
    
    int pos[6]={};
    for (int i = 1; i < 7; i++){
      int16_t val = 0;
      if (packet_handler->readPos(i, val))
      {
        float pos_angle = feetech_sts_interface::STS3032::data2angle(val);
        std::cout << "pos: ";
        std::cout << std::fixed << std::setprecision(3) << pos_angle;
        std::cout << " deg" << std::endl;
        pos[i-1] = roundf(pos_angle/45*512);
      }
      else
      {
        std::cout << "failed to read pos" << std::endl;
      }
    }

    msg.id_1 = 11;
    msg.id_2 = 12;
    msg.id_3 = 13;
    msg.id_4 = 14;
    msg.id_5 = 15;
    msg.id_6 = 16;
    msg.position_1 = pos[0];
    msg.position_2 = pos[1];
    msg.position_3 = pos[2];
    msg.position_4 = pos[3];
    msg.position_5 = pos[4];
    msg.position_6 = pos[5];
    publisher_six_motor_present_position->publish(msg);
  });

}

PubFeetechNode::~PubFeetechNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down");
  auto port_handler = std::make_shared<h6x_serial_interface::PortHandler>("/dev/ttyUSB0");
  port_handler->close();
}
}  // namespace feetech_sts_interface

RCLCPP_COMPONENTS_REGISTER_NODE(feetech_sts_interface::PubFeetechNode)
