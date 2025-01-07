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

#include <feetech_sts_interface/feetech_sts_interface.hpp>
#include <unistd.h>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "feetech_sts_example/read_pos.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position_six_motor.hpp"

using namespace feetech_sts_interface;

PubFeetechNode::PubFeetechNode()
: Node("pub_feetech_node")
{
  RCLCPP_INFO(this->get_logger(), "Run read write node");
  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();
    publisher_six_motor_present_position_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPositionSixMotor>("/six_motor_present_position", 10);
}



int main(int argc, char ** argv)
{
  if (argc != 4) {
    std::cout << "Usage: " << argv[0] << " <id (0)> <port_name (/dev/ttyUSB0)> <baudrate (1000000)>" << std::endl;
    return EXIT_FAILURE;
  }
  int TARGET_ID = std::stoi(argv[1]);
  std::string port_name = argv[2];
  int baudrate = std::stoi(argv[3]);

  auto port_handler = std::make_shared<h6x_serial_interface::PortHandler>(port_name);
  auto packet_handler = std::make_shared<feetech_sts_interface::PacketHandler>(port_handler);

  port_handler->configure(baudrate);
  if (!port_handler->open()) {
    return EXIT_FAILURE;
  }

  packet_handler->setTorque(TARGET_ID, 0);

  SetPositionSixMotor message;

  unsigned char* id[] = { &message.id_1, &message.id_2, &message.id_3 ,&message.id_4, &message.id_5, &message.id_6 };

  //int msg.position_1, msg.position_2, msg.position_3, msg.position_4, msg.position_5;
  int* position[] = { &message.position_1, &message.position_2, &message.position_3, &message.position_4, &message.position_5, &message.position_6 };
  uint32_t present; 

  using namespace std::chrono_literals;  // NOLINT
  while (true)
  {
    for (int i = 1; i < 7; i++)
    {

      int16_t val = 0;
      if (packet_handler->readPos(i, val))
      {
        float pos_angle = feetech_sts_interface::STS3032::data2angle(val);
        std::cout << "id: " << i << " ";
        std::cout << "pos: ";
        std::cout << std::fixed << std::setprecision(3) << pos_angle;
        std::cout << " deg" << std::endl;
        *id[i-11] = static_cast<unsigned char>(i);
        *position[i-11] = static_cast<int>(present);
      }
      else
      {
        std::cout << "failed to read pos" << std::endl;
      }
    }
    publisher_six_motor_present_position_ ->publish(message);
    std::this_thread::sleep_for(5ms);
  }

  port_handler->close();
  return EXIT_FAILURE;
}
