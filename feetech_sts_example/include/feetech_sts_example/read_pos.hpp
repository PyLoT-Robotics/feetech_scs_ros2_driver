#ifndef READ_WRITE_NODE_HPP_
#define READ_WRITE_NODE_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rcutils/cmdline_parser.h"

#include <rclcpp/rclcpp.hpp>
#include "dynamixel_sdk_custom_interfaces/msg/set_position_six_motor.hpp"

class PubFeetechNode : public rclcpp::Node
{
public:
    using SetPositionSixMotor = dynamixel_sdk_custom_interfaces::msg::SetPositionSixMotor;
    PubFeetechNode();

private:
    rclcpp::Publisher<SetPositionSixMotor>::SharedPtr publisher_six_motor_present_position_;
};