#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include <feetech_sts_interface/feetech_sts_interface.hpp>

class PantiltReadWriteNode : public rclcpp::Node
{
public:
  using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
  using GetPosition = dynamixel_sdk_custom_interfaces::srv::GetPosition;

  PantiltReadWriteNode(
    std::shared_ptr<h6x_serial_interface::PortHandler> port_handler,
    std::shared_ptr<feetech_sts_interface::PacketHandler> packet_handler);
  ~PantiltReadWriteNode();

private:
  void publish_position_callback();

  std::shared_ptr<h6x_serial_interface::PortHandler> port_handler_;
  std::shared_ptr<feetech_sts_interface::PacketHandler> packet_handler_;

  rclcpp::Subscription<SetPosition>::SharedPtr set_position_subscriber_;
  rclcpp::Service<GetPosition>::SharedPtr get_position_server_;
  rclcpp::Publisher<SetPosition>::SharedPtr position_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
