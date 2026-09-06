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

// Read-only joint state publisher for the SO-100 arm.
//
// This node only reads servo positions (it never enables/disables torque or
// writes commands), and publishes them on /joint_states_so100 instead of the
// standard /joint_states topic so it never collides with the joint_states
// coming from the arm actually being teleoperated.

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <feetech_sts_interface/feetech_sts_interface.hpp>
#include <h6x_serial_interface/port_handler.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using sensor_msgs::msg::JointState;

namespace
{
constexpr u_char kMinServoId = 1;
constexpr u_char kMaxServoId = 6;

// SO-100 arm joint names, indexed by servo ID (1-6).
const std::map<u_char, std::string> kJointNames = {
  {1, "shoulder_pan"},
  {2, "shoulder_lift"},
  {3, "elbow_flex"},
  {4, "wrist_flex"},
  {5, "wrist_roll"},
  {6, "gripper"},
};
}  // namespace

class So100JointStatePublisher : public rclcpp::Node
{
public:
  So100JointStatePublisher(
    std::shared_ptr<feetech_sts_interface::PacketHandler> packet_handler,
    std::vector<u_char> servo_ids,
    double publish_rate)
  : Node("so100_joint_state_publisher"),
    packet_handler_(packet_handler),
    servo_ids_(std::move(servo_ids))
  {
    joint_state_pub_ = this->create_publisher<JointState>(
      "/joint_states_so100", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    auto timer_period =
      std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
    timer_ = this->create_wall_timer(
      timer_period,
      std::bind(&So100JointStatePublisher::publishCurrentState, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Publishing %zu joint(s) to /joint_states_so100 at %.1f Hz",
      servo_ids_.size(), publish_rate);
  }

private:
  void publishCurrentState()
  {
    auto msg = JointState();
    msg.header.stamp = this->now();

    for (const auto & id : servo_ids_) {
      int16_t pos_data = 0;
      if (!packet_handler_->readPos(id, pos_data)) {
        RCLCPP_WARN(this->get_logger(), "Failed to read position for servo ID %d", id);
        continue;
      }

      double angle_deg = feetech_sts_interface::STS3032::data2angle(pos_data);
      double angle_rad = angle_deg * M_PI / 180.0;

      msg.name.push_back(kJointNames.at(id));
      msg.position.push_back(angle_rad);
    }

    if (!msg.name.empty()) {
      joint_state_pub_->publish(msg);
    }
  }

  rclcpp::Publisher<JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<feetech_sts_interface::PacketHandler> packet_handler_;
  std::vector<u_char> servo_ids_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc != 4) {
    std::cout << "Usage: " << argv[0]
               << " <port_name (/dev/ttyUSB0)> <baudrate (1000000)> <publish_rate_hz (30)>"
               << std::endl;
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  std::string port_name = argv[1];
  int baudrate = std::stoi(argv[2]);
  double publish_rate = std::stod(argv[3]);

  auto port_handler = std::make_shared<h6x_serial_interface::PortHandler>(port_name);
  auto packet_handler =
    std::make_shared<feetech_sts_interface::PacketHandler>(port_handler);

  port_handler->configure(baudrate);
  if (!port_handler->open()) {
    RCLCPP_ERROR(rclcpp::get_logger("so100_joint_state_publisher"), "Failed to open port");
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  // Discover which of the SO-100's 6 servos actually respond, read-only
  // (no torque enable/disable — this node never actuates the arm).
  std::vector<u_char> servo_ids;
  for (u_char id = kMinServoId; id <= kMaxServoId; ++id) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (packet_handler->ping(id) > 0) {
      RCLCPP_INFO(
        rclcpp::get_logger("so100_joint_state_publisher"),
        "Found servo ID %d (%s)", id, kJointNames.at(id).c_str());
      servo_ids.push_back(id);
    }
  }

  if (servo_ids.empty()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("so100_joint_state_publisher"),
      "No servos found on %s", port_name.c_str());
    port_handler->close();
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  auto node = std::make_shared<So100JointStatePublisher>(
    packet_handler, servo_ids, publish_rate);

  rclcpp::spin(node);

  port_handler->close();
  rclcpp::shutdown();
  return 0;
}