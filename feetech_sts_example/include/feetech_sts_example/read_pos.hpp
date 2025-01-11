#ifndef READ_WRITE_NODEHPP
#define READ_WRITE_NODEHPP

#include <cstdio>
#include <memory>
#include <string>

#include "rcutils/cmdline_parser.h"

#include <rclcpp/rclcpp.hpp>
#include "dynamixel_sdk_custom_interfaces/msg/set_position_six_motor.hpp"
#include "feetech_sts_example/visibility_control.h"
#include "h6x_serial_interface/port_handler.hpp"

namespace feetech_sts_interface
{
  

class PubFeetechNode : public rclcpp::Node
{
public:
    using SetPositionSixMotor = dynamixel_sdk_custom_interfaces::msg::SetPositionSixMotor;
    explicit PubFeetechNode(const rclcpp::NodeOptions & options);
    virtual ~PubFeetechNode();



private:
    rclcpp::Publisher<SetPositionSixMotor>::SharedPtr publisher_six_motor_present_position;
    rclcpp::TimerBase::SharedPtr timer_;
    feetech_sts_interface::PacketHandler* packet_handler;
    feetech_sts_interface::PacketHandler* port_handler;
};

}  // namespace feetech_sts_interface

#endif  // JOINT_PUB_NODEHPP