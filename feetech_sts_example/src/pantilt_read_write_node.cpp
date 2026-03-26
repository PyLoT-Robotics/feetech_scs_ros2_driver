// Feetech STS Pan-Tilt read/write node
// Usage:
//   ros2 run feetech_sts_example pantilt_read_write_node --ros-args -p port_name:=/dev/ttyACM3 -p baudrate:=1000000
//
// Write position:
//   ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/SetPosition "{id: 1, position: 4093}"
//
// Read position:
//   ros2 service call /get_position dynamixel_sdk_custom_interfaces/srv/GetPosition "id: 1"
//
// Position is published on /present_position at 10Hz

#include "feetech_sts_example/pantilt_read_write_node.hpp"

// Servo IDs
#define TILT_ID  1
#define PAN_ID   2

// ID1 (tilt): 1200 = origin
#define TILT_ORIGIN    1050
#define TILT_LIMIT_MIN 0
#define TILT_LIMIT_MAX 1200

// ID2 (pan): 1900 = origin
#define PAN_ORIGIN    1900

// Default speed and acceleration for position writes
#define DEFAULT_SPEED 500
#define DEFAULT_ACC   50

PantiltReadWriteNode::PantiltReadWriteNode(
  std::shared_ptr<h6x_serial_interface::PortHandler> port_handler,
  std::shared_ptr<feetech_sts_interface::PacketHandler> packet_handler)
: Node("pantilt_read_write_node"),
  port_handler_(port_handler),
  packet_handler_(packet_handler)
{
  RCLCPP_INFO(this->get_logger(), "Starting Feetech pan-tilt read/write node");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  // Enable torque on both servos
  packet_handler_->setTorque(TILT_ID, true);
  packet_handler_->setTorque(PAN_ID, true);
  RCLCPP_INFO(this->get_logger(), "Torque enabled on ID %d and ID %d", TILT_ID, PAN_ID);

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
      if (id == TILT_ID) {
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

  RCLCPP_INFO(this->get_logger(), "Pan-tilt node ready. Tilt(ID%d) origin=%d limit=[%d, %d], Pan(ID%d) origin=%d",
    TILT_ID, TILT_ORIGIN, TILT_LIMIT_MIN, TILT_LIMIT_MAX, PAN_ID, PAN_ORIGIN);
}

PantiltReadWriteNode::~PantiltReadWriteNode()
{
}

void PantiltReadWriteNode::publish_position_callback()
{
  for (int id = TILT_ID; id <= PAN_ID; ++id) {
    int16_t pos = 0;
    if (packet_handler_->readPos(static_cast<uint8_t>(id), pos)) {
      auto msg = SetPosition();
      msg.id = static_cast<uint8_t>(id);
      msg.position = static_cast<int32_t>(pos);
      position_publisher_->publish(msg);
    }
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create a temporary node to read parameters before constructing the main node
  auto param_node = std::make_shared<rclcpp::Node>("pantilt_param_reader");
  param_node->declare_parameter("port_name", "/dev/ttyACM3");
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

  // Disable torque on shutdown
  packet_handler->setTorque(1, false);
  packet_handler->setTorque(2, false);
  RCLCPP_INFO(rclcpp::get_logger("pantilt_read_write_node"), "Torque disabled");

  port_handler->close();
  rclcpp::shutdown();
  return 0;
}
