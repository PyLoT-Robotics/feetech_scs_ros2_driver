/**
 * @file pantilt_read_write_node.cpp
 * @brief Feetech STS3215 Pan-Tilt servo read/write implementation
 * 
 * Device Setup:
 *   - USB Controller: CH340 (VID:PID 1a86:55d3)
 *   - Servo Control: Feetech STS3215
 *   - Default port: /dev/ttyUSB_sts3215 (requires udev rule)
 *   - Speed: 1000000 baud (1 Mbps)
 * 
 * udev Installation:
 *   The feetech_sts_interface package includes an automated setup script:
 *   
 *   sudo sh -c 'cat > /etc/udev/rules.d/99-sts3215.rules << EOF
 *   # Feetech STS3215 servo control board (CH340)
 *   SUBSYSTEMS=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", MODE="0666", SYMLINK+="ttyUSB_sts3215"
 *   SUBSYSTEMS=="usb-serial", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", MODE="0666", GROUP="dialout"
 *   EOF'
 *   
 *   sudo udevadm control --reload-rules
 *   sudo udevadm trigger
 * 
 *   Or run:
 *   sudo ./install/feetech_sts_interface/share/feetech_sts_interface/scripts/install_udev_rules.sh
 * 
 * Usage Example:
 *   ros2 run feetech_sts_example pantilt_read_write_node --ros-args -p port_name:=/dev/ttyUSB_sts3215 -p baudrate:=1000000
 *
 * Write position:
 *   ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/SetPosition "{id: 1, position: 4093}"
 *
 * Read position:
 *   ros2 service call /get_position dynamixel_sdk_custom_interfaces/srv/GetPosition "id: 1"
 *
 * Position is published on /present_position at 10Hz
 */

#include "feetech_sts_example/pantilt_read_write_node.hpp"

#include <algorithm>
#include <sstream>
#include <thread>
#include <vector>

// ID1 (tilt): 1200 = origin
#define TILT_ORIGIN    1050
#define TILT_LIMIT_MIN 0
#define TILT_LIMIT_MAX 1200

// ID2 (pan): 1900 = origin
#define PAN_ORIGIN    1900

// Default speed and acceleration for position writes
#define DEFAULT_SPEED 2000
#define DEFAULT_ACC   500

PantiltReadWriteNode::PantiltReadWriteNode(
  std::shared_ptr<h6x_serial_interface::PortHandler> port_handler,
  std::shared_ptr<feetech_sts_interface::PacketHandler> packet_handler)
: Node("pantilt_read_write_node"),
  port_handler_(port_handler),
  packet_handler_(packet_handler)
{
  RCLCPP_INFO(this->get_logger(), "Starting Feetech pan-tilt read/write node");

  this->declare_parameter("qos_depth", 10);
  this->declare_parameter("tilt_id", 1);
  this->declare_parameter("pan_id", 2);
  this->declare_parameter("scan_id_start", 1);
  this->declare_parameter("scan_id_end", 20);
  this->declare_parameter("read_retry_count", 3);
  this->declare_parameter("read_retry_interval_ms", 5);

  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  int tilt_id_param = this->get_parameter("tilt_id").as_int();
  int pan_id_param = this->get_parameter("pan_id").as_int();
  int scan_id_start = this->get_parameter("scan_id_start").as_int();
  int scan_id_end = this->get_parameter("scan_id_end").as_int();
  int read_retry_count = this->get_parameter("read_retry_count").as_int();
  int read_retry_interval_ms = this->get_parameter("read_retry_interval_ms").as_int();
  read_retry_count = std::max(1, read_retry_count);
  read_retry_interval_ms = std::max(0, read_retry_interval_ms);
  if (tilt_id_param < 0 || tilt_id_param > 253 || pan_id_param < 0 || pan_id_param > 253) {
    RCLCPP_WARN(this->get_logger(), "Invalid servo IDs (tilt=%d, pan=%d), fallback to 1/2",
      tilt_id_param, pan_id_param);
    tilt_id_param = 1;
    pan_id_param = 2;
  }
  if (tilt_id_param == pan_id_param) {
    RCLCPP_WARN(this->get_logger(), "tilt_id and pan_id are identical (%d), fallback to 1/2", tilt_id_param);
    tilt_id_param = 1;
    pan_id_param = 2;
  }
  tilt_id_ = static_cast<uint8_t>(tilt_id_param);
  pan_id_ = static_cast<uint8_t>(pan_id_param);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  // Enable torque on both servos and report if command itself fails.
  const bool tilt_torque_ok = packet_handler_->setTorque(tilt_id_, true);
  const bool pan_torque_ok = packet_handler_->setTorque(pan_id_, true);
  if (!tilt_torque_ok || !pan_torque_ok) {
    RCLCPP_ERROR(this->get_logger(),
      "Failed to send torque command (tilt_id=%u ok=%d, pan_id=%u ok=%d)",
      tilt_id_, tilt_torque_ok, pan_id_, pan_torque_ok);
  } else {
    RCLCPP_INFO(this->get_logger(), "Torque command sent on ID %u and ID %u", tilt_id_, pan_id_);
  }

  const auto read_pos_with_retry =
    [this, read_retry_count, read_retry_interval_ms](uint8_t id, int16_t & pos) -> bool {
      for (int i = 0; i < read_retry_count; ++i) {
        if (packet_handler_->readPos(id, pos)) {
          return true;
        }
        if (read_retry_interval_ms > 0) {
          std::this_thread::sleep_for(std::chrono::milliseconds(read_retry_interval_ms));
        }
      }
      return false;
    };

  int16_t tilt_pos = 0;
  int16_t pan_pos = 0;
  const bool tilt_read_ok = read_pos_with_retry(tilt_id_, tilt_pos);
  const bool pan_read_ok = read_pos_with_retry(pan_id_, pan_pos);

  if (!tilt_read_ok && !pan_read_ok) {
    RCLCPP_ERROR(this->get_logger(),
      "No servo response after retries. Check ID/baudrate/wiring (tilt_id=%u, pan_id=%u)",
      tilt_id_, pan_id_);

    if (scan_id_start > scan_id_end) {
      std::swap(scan_id_start, scan_id_end);
    }
    scan_id_start = std::max(1, scan_id_start);
    scan_id_end = std::min(253, scan_id_end);

    std::vector<int> alive_ids;
    for (int id = scan_id_start; id <= scan_id_end; ++id) {
      int16_t scan_pos = 0;
      if (packet_handler_->readPos(static_cast<uint8_t>(id), scan_pos)) {
        alive_ids.push_back(id);
      }
    }

    if (alive_ids.empty()) {
      RCLCPP_ERROR(this->get_logger(),
        "ID scan [%d..%d]: no response. Likely wiring/power/baudrate issue.",
        scan_id_start, scan_id_end);
    } else {
      std::ostringstream oss;
      for (size_t i = 0; i < alive_ids.size(); ++i) {
        oss << alive_ids[i];
        if (i + 1 < alive_ids.size()) {
          oss << ",";
        }
      }
      RCLCPP_WARN(this->get_logger(),
        "Detected responding servo IDs in scan [%d..%d]: %s. Update tilt_id/pan_id accordingly.",
        scan_id_start, scan_id_end, oss.str().c_str());
    }
  } else if (!tilt_read_ok || !pan_read_ok) {
    RCLCPP_WARN(this->get_logger(),
      "Partial response: tilt_id=%u ok=%d pos=%d, pan_id=%u ok=%d pos=%d",
      tilt_id_, tilt_read_ok, tilt_pos, pan_id_, pan_read_ok, pan_pos);
  } else {
    RCLCPP_INFO(this->get_logger(), "Servo response OK: tilt=%d pan=%d", tilt_pos, pan_pos);
  }

  // Subscriber: /set_position
  set_position_subscriber_ =
    this->create_subscription<SetPosition>(
    "set_position",
    QOS_RKL10V,
    [this](const SetPosition::SharedPtr msg) -> void
    {
      uint8_t id = static_cast<uint8_t>(msg->id);
      int16_t position = static_cast<int16_t>(msg->position);

      // Clamp tilt position within min/max limits
      if (id == tilt_id_) {
        if (position < TILT_LIMIT_MIN) position = TILT_LIMIT_MIN;
        if (position > TILT_LIMIT_MAX) position = TILT_LIMIT_MAX;
      }

      u_char ids[1] = {id};
      int16_t positions[1] = {position};
      int16_t speeds[1] = {DEFAULT_SPEED};
      u_short accs[1] = {DEFAULT_ACC};

      bool ok = packet_handler_->syncWritePosEx(ids, 1, positions, speeds, accs);

      if (ok) {
        RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", msg->id, msg->position);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to write position to ID %d", msg->id);
      }
    }
  );

  // Service: /get_position
  auto get_present_position =
    [this](
    const std::shared_ptr<GetPosition::Request> request,
    std::shared_ptr<GetPosition::Response> response) -> void
    {
      int16_t pos = 0;
      bool ok = packet_handler_->readPos(static_cast<uint8_t>(request->id), pos);

      if (ok) {
        response->position = static_cast<int32_t>(pos);
        RCLCPP_INFO(this->get_logger(), "Get [ID: %d] [Present Position: %d]", request->id, pos);
      } else {
        response->position = -1;
        RCLCPP_ERROR(this->get_logger(), "Failed to read position from ID %d", request->id);
      }
    };

  get_position_server_ = create_service<GetPosition>("get_position", get_present_position);

  // Publisher: /present_position at 10Hz
  position_publisher_ = this->create_publisher<SetPosition>("present_position", QOS_RKL10V);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&PantiltReadWriteNode::publish_position_callback, this));

  RCLCPP_INFO(this->get_logger(),
    "Pan-tilt node ready. Tilt(ID%u) origin=%d limit=[%d, %d], Pan(ID%u) origin=%d",
    tilt_id_, TILT_ORIGIN, TILT_LIMIT_MIN, TILT_LIMIT_MAX, pan_id_, PAN_ORIGIN);
}

PantiltReadWriteNode::~PantiltReadWriteNode()
{
  const bool tilt_off_ok = packet_handler_->setTorque(tilt_id_, false);
  const bool pan_off_ok = packet_handler_->setTorque(pan_id_, false);
  if (!tilt_off_ok || !pan_off_ok) {
    RCLCPP_WARN(this->get_logger(),
      "Failed to send torque-off command (tilt_id=%u ok=%d, pan_id=%u ok=%d)",
      tilt_id_, tilt_off_ok, pan_id_, pan_off_ok);
  }
}

void PantiltReadWriteNode::publish_position_callback()
{
  for (const uint8_t id : {tilt_id_, pan_id_}) {
    int16_t pos = 0;
    if (packet_handler_->readPos(static_cast<uint8_t>(id), pos)) {
      auto msg = SetPosition();
      msg.id = static_cast<uint8_t>(id);
      msg.position = static_cast<int32_t>(pos);
      position_publisher_->publish(msg);
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Failed to read present position from ID %u", id);
    }
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create a temporary node to read parameters before constructing the main node
  auto param_node = std::make_shared<rclcpp::Node>("pantilt_param_reader");
  param_node->declare_parameter("port_name", "/dev/ttyUSB_sts3215");
  param_node->declare_parameter("baudrate", 1000000);

  std::string port_name = param_node->get_parameter("port_name").as_string();
  int baudrate = param_node->get_parameter("baudrate").as_int();
  param_node.reset();

  // Open serial port
  auto port_handler = std::make_shared<h6x_serial_interface::PortHandler>(port_name);
  auto packet_handler = std::make_shared<feetech_sts_interface::PacketHandler>(port_handler);

  port_handler->configure(baudrate);
  if (!port_handler->open()) {
    RCLCPP_ERROR(rclcpp::get_logger("pantilt_read_write_node"), "Failed to open port: %s", port_name.c_str());
    rclcpp::shutdown();
    return -1;
  }
  RCLCPP_INFO(rclcpp::get_logger("pantilt_read_write_node"), "Opened port: %s at %d baud", port_name.c_str(), baudrate);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  auto node = std::make_shared<PantiltReadWriteNode>(port_handler, packet_handler);
  rclcpp::spin(node);

  node.reset();
  RCLCPP_INFO(rclcpp::get_logger("pantilt_read_write_node"), "Node stopped");

  port_handler->close();
  rclcpp::shutdown();
  return 0;
}
