#pragma once

/**
 * @file pantilt_read_write_node.hpp
 * @brief Feetech STS3215 Pan-Tilt servo control node
 * 
 * Device Setup:
 *   - Chip: CH340 USB-to-Serial adapter
 *   - USB ID: 1a86:55d3 (QinHeng Electronics)
 *   - Default symlink: /dev/ttyUSB_sts3215
 *   - Baud rate: 1000000 (1 Mbps)
 * 
 * udev Configuration:
 *   To set up automatic device recognition, install the udev rule:
 *   
 *   sudo sh -c 'cat > /etc/udev/rules.d/99-sts3215.rules << EOF
 *   # Feetech STS3215 servo control board
 *   SUBSYSTEMS=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", MODE="0666", SYMLINK+="ttyUSB_sts3215"
 *   SUBSYSTEMS=="usb-serial", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", MODE="0666", GROUP="dialout"
 *   EOF'
 *   
 *   sudo udevadm control --reload-rules
 *   sudo udevadm trigger
 * 
 *   Or use the automated script:
 *   sudo ./install/feetech_sts_interface/share/feetech_sts_interface/scripts/install_udev_rules.sh
 */

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

  uint8_t tilt_id_{1};
  uint8_t pan_id_{2};

  std::shared_ptr<h6x_serial_interface::PortHandler> port_handler_;
  std::shared_ptr<feetech_sts_interface::PacketHandler> packet_handler_;

  rclcpp::Subscription<SetPosition>::SharedPtr set_position_subscriber_;
  rclcpp::Service<GetPosition>::SharedPtr get_position_server_;
  rclcpp::Publisher<SetPosition>::SharedPtr position_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
